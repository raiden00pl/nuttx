/****************************************************************************
 * arch/arm/src/stm32n6/stm32_mdf.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* MDF1 digital microphone capture.
 *
 * Filter 0 decimates the PDM bitstream of one microphone:
 *
 *   MDF1_SDI0 (sampled on the rising edge of MDF1_CCK0) -> bitstream
 *   matrix -> Sinc4 CIC -> reshape filter (/4) -> high-pass filter ->
 *   RXFIFO -> GPDMA -> audio buffer
 *
 * The filter settings follow the ST STM32N6570-DK BSP.  The filter output
 * is 24 bits, left aligned in the 32-bit data register.  The GPDMA moves
 * whole words into the audio buffer, one transfer per buffer, and the
 * worker converts them in place to saturated 16-bit PCM, duplicating the
 * sample for stereo streams.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/param.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/cache.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/nuttx.h>
#include <nuttx/queue.h>
#include <nuttx/wqueue.h>
#include <nuttx/audio/audio.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "stm32.h"
#include "hardware/stm32n6xxx_mdf.h"
#include "stm32_mdf.h"

#ifdef CONFIG_STM32_MDF1

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_AUDIO
#  error CONFIG_AUDIO required by this driver
#endif

#ifndef CONFIG_SCHED_HPWORK
#  error High priority work queue support is required (CONFIG_SCHED_HPWORK)
#endif

#ifndef CONFIG_STM32_DMA1
#  error GPDMA1 is required by this driver (CONFIG_STM32_DMA1)
#endif

#ifndef STM32_MDF1_FREQUENCY
#  error "Please define STM32_MDF1_FREQUENCY in board.h"
#endif

/* Filter 0 registers */

#define MDF_FLT_BASE        STM32_MDF1_FLT_BASE(0)
#define MDF_SITFCR          (MDF_FLT_BASE + STM32_MDF_SITFCR_OFFSET)
#define MDF_BSMXCR          (MDF_FLT_BASE + STM32_MDF_BSMXCR_OFFSET)
#define MDF_DFLTCR          (MDF_FLT_BASE + STM32_MDF_DFLTCR_OFFSET)
#define MDF_DFLTCICR        (MDF_FLT_BASE + STM32_MDF_DFLTCICR_OFFSET)
#define MDF_DFLTRSFR        (MDF_FLT_BASE + STM32_MDF_DFLTRSFR_OFFSET)
#define MDF_DFLTINTR        (MDF_FLT_BASE + STM32_MDF_DFLTINTR_OFFSET)
#define MDF_DLYCR           (MDF_FLT_BASE + STM32_MDF_DLYCR_OFFSET)
#define MDF_DFLTIER         (MDF_FLT_BASE + STM32_MDF_DFLTIER_OFFSET)
#define MDF_DFLTISR         (MDF_FLT_BASE + STM32_MDF_DFLTISR_OFFSET)
#define MDF_OECCR           (MDF_FLT_BASE + STM32_MDF_OECCR_OFFSET)
#define MDF_DFLTDR          (MDF_FLT_BASE + STM32_MDF_DFLTDR_OFFSET)

/* The reshape filter decimates by 4 after the Sinc4 filter */

#define MDF_RSF_DECIMATION  4

/* Largest MDF_CCK0 divider from the processing clock */

#define MDF_CCKDIV_MAX      16

/* Serial interface clock absence threshold (ST BSP value) */

#define MDF_SITF_THRESHOLD  31

/* Filter output samples dropped after each start while the microphone
 * wakes up and the filters settle.
 */

#define MDF_DISCARD         255

/* The output still swings to full scale for about 30 ms after the
 * discarded samples, so mute the beginning of each capture.
 */

#define MDF_MUTE_MS         64

/* Polling loops to wait for an active flag to clear */

#define MDF_TIMEOUT         100000

/* Receive buffers are aligned to the Cortex-M55 D-cache line so that
 * their cache maintenance does not touch neighbouring data.
 */

#define MDF_BUFALIGN        32

/* The GPDMA block size is in bytes and limited to 16 bits */

#define MDF_DMA_MAXBYTES    (UINT16_MAX & ~3)

/* Word transfers from the data register to the AXI memory port */

#define MDF_DMA_CONFIG      (GPDMA_CXTR1_SDW_LOG2_WORD | \
                             GPDMA_CXTR1_DDW_LOG2_WORD | \
                             GPDMA_CXTR1_DINC | GPDMA_CXTR1_DAP)

#define MDF_DEFAULT_RATE    16000

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Filter settings for one sample rate.  The microphone clock is
 * rate x mcicd x MDF_RSF_DECIMATION: 2.048 MHz up to 32 kHz and 3.072 MHz
 * at 48 kHz.
 */

struct mdf_rate_s
{
  uint32_t rate;                      /* Sample rate in Hz */
  uint16_t mcicd;                     /* Sinc4 decimation ratio */
  int8_t   gain;                      /* Scale, steps of about 3 dB */
};

struct stm32_mdf_s
{
  struct audio_lowerhalf_s dev;       /* Audio lower half (must be first) */
  DMA_HANDLE dma;                     /* GPDMA channel */
  struct work_s work;                 /* Buffer completion work */
  struct dq_queue_s pendq;            /* Buffers waiting for the DMA */
  struct dq_queue_s doneq;            /* Buffers filled by the DMA */
  FAR struct ap_buffer_s *act;        /* Buffer being filled */
  FAR const struct mdf_rate_s *rate;  /* Current sample rate settings */
  uint32_t mute;                      /* Samples left to mute */
  uint8_t nchannels;                  /* 1 (mono) or 2 (duplicated) */
  bool reserved;                      /* A session is reserved */
  bool running;                       /* The filter is acquiring */
  bool stopping;                      /* Buffers must be returned */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void mdf_dmacallback(DMA_HANDLE handle, uint8_t status, void *arg);
static void mdf_worker(FAR void *arg);

static int mdf_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                       FAR struct audio_caps_s *caps);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int mdf_configure(FAR struct audio_lowerhalf_s *dev,
                         FAR void *session,
                         FAR const struct audio_caps_s *caps);
#else
static int mdf_configure(FAR struct audio_lowerhalf_s *dev,
                         FAR const struct audio_caps_s *caps);
#endif
static int mdf_shutdown(FAR struct audio_lowerhalf_s *dev);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int mdf_start(FAR struct audio_lowerhalf_s *dev, FAR void *session);
#else
static int mdf_start(FAR struct audio_lowerhalf_s *dev);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int mdf_stop(FAR struct audio_lowerhalf_s *dev, FAR void *session);
#else
static int mdf_stop(FAR struct audio_lowerhalf_s *dev);
#endif
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int mdf_pause(FAR struct audio_lowerhalf_s *dev, FAR void *session);
static int mdf_resume(FAR struct audio_lowerhalf_s *dev,
                      FAR void *session);
#else
static int mdf_pause(FAR struct audio_lowerhalf_s *dev);
static int mdf_resume(FAR struct audio_lowerhalf_s *dev);
#endif
#endif
static int mdf_allocbuffer(FAR struct audio_lowerhalf_s *dev,
                           FAR struct audio_buf_desc_s *bufdesc);
static int mdf_freebuffer(FAR struct audio_lowerhalf_s *dev,
                          FAR struct audio_buf_desc_s *bufdesc);
static int mdf_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                             FAR struct ap_buffer_s *apb);
static int mdf_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                     unsigned long arg);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int mdf_reserve(FAR struct audio_lowerhalf_s *dev,
                       FAR void **session);
static int mdf_release(FAR struct audio_lowerhalf_s *dev,
                       FAR void *session);
#else
static int mdf_reserve(FAR struct audio_lowerhalf_s *dev);
static int mdf_release(FAR struct audio_lowerhalf_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ST BSP decimation ratios and gains.  The output of the 24-bit filter is
 * scaled so that the microphone full scale is about 2^21.
 */

static const struct mdf_rate_s g_mdf_rates[] =
{
  {  8000, 64, -4 },
  { 16000, 32,  2 },
  { 32000, 16, 10 },
  { 48000, 16, 10 },
};

static const struct audio_ops_s g_mdf_ops =
{
  .getcaps       = mdf_getcaps,
  .configure     = mdf_configure,
  .shutdown      = mdf_shutdown,
  .start         = mdf_start,
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  .stop          = mdf_stop,
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
  .pause         = mdf_pause,
  .resume        = mdf_resume,
#endif
  .allocbuffer   = mdf_allocbuffer,
  .freebuffer    = mdf_freebuffer,
  .enqueuebuffer = mdf_enqueuebuffer,
  .ioctl         = mdf_ioctl,
  .reserve       = mdf_reserve,
  .release       = mdf_release,
};

static struct stm32_mdf_s g_mdf1 =
{
  .dev =
    {
      .ops = &g_mdf_ops,
    },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mdf_waitclear
 *
 * Description:
 *   Wait until an active flag clears after its enable bit was cleared.
 *
 ****************************************************************************/

static void mdf_waitclear(uint32_t addr, uint32_t flag)
{
  int timeout;

  for (timeout = 0; timeout < MDF_TIMEOUT; timeout++)
    {
      if ((getreg32(addr) & flag) == 0)
        {
          return;
        }
    }

  auderr("ERROR: Timeout at %08" PRIx32 "\n", addr);
}

/****************************************************************************
 * Name: mdf_callback
 *
 * Description:
 *   Report a buffer or the end of the stream to the upper half.
 *
 ****************************************************************************/

static void mdf_callback(FAR struct stm32_mdf_s *priv, uint16_t reason,
                         FAR struct ap_buffer_s *apb)
{
#ifdef CONFIG_AUDIO_MULTI_SESSION
  priv->dev.upper(priv->dev.priv, reason, apb, OK, NULL);
#else
  priv->dev.upper(priv->dev.priv, reason, apb, OK);
#endif
}

/****************************************************************************
 * Name: mdf_dmabytes
 *
 * Description:
 *   Return the number of bytes the DMA writes into a buffer: whole 32-bit
 *   samples within the 16-bit GPDMA block size.
 *
 ****************************************************************************/

static apb_samp_t mdf_dmabytes(FAR struct ap_buffer_s *apb)
{
  return MIN(apb->nmaxbytes & ~3, MDF_DMA_MAXBYTES);
}

/****************************************************************************
 * Name: mdf_hwstart
 *
 * Description:
 *   Start the microphone clock and the serial interface and configure
 *   filter 0 for the current sample rate, following the sequence of the
 *   ST HAL (HAL_MDF_Init() and MDF_AcqStart()).  The filter itself is
 *   enabled by the caller once the DMA is armed.
 *
 ****************************************************************************/

static void mdf_hwstart(FAR struct stm32_mdf_s *priv)
{
  FAR const struct mdf_rate_s *rate = priv->rate;
  uint32_t pdmclk;
  uint32_t procdiv;
  uint32_t cckdiv;
  uint32_t scale;
  uint32_t div;

  /* Kernel clock -> processing clock (PROCDIV) -> MDF_CCK0 (CCKDIV) */

  pdmclk  = rate->rate * rate->mcicd * MDF_RSF_DECIMATION;
  div     = STM32_MDF1_FREQUENCY / pdmclk;
  procdiv = (div + MDF_CCKDIV_MAX - 1) / MDF_CCKDIV_MAX;
  cckdiv  = div / procdiv;

  if (procdiv * cckdiv * pdmclk != STM32_MDF1_FREQUENCY)
    {
      audwarn("WARNING: MDF_CCK0 %" PRIu32 " Hz instead of %" PRIu32 "\n",
              STM32_MDF1_FREQUENCY / (procdiv * cckdiv), pdmclk);
    }

  audinfo("rate %" PRIu32 ": PROCDIV %" PRIu32 " CCKDIV %" PRIu32 "\n",
          rate->rate, procdiv, cckdiv);

  /* Clock generator: MDF_CCK0 is an output clocking the microphone */

  putreg32(0, STM32_MDF1_CKGCR);
  mdf_waitclear(STM32_MDF1_CKGCR, MDF_CKGCR_CCKACTIVE);

  putreg32(MDF_CKGCR_PROCDIV(procdiv) | MDF_CKGCR_CCKDIV(cckdiv) |
           MDF_CKGCR_CCK0DIR | MDF_CKGCR_CCK0EN, STM32_MDF1_CKGCR);
  modifyreg32(STM32_MDF1_CKGCR, 0, MDF_CKGCR_CKDEN);

  /* Serial interface 0: normal SPI mode clocked by MDF_CCK0 */

  putreg32(0, MDF_SITFCR);
  mdf_waitclear(MDF_SITFCR, MDF_SITFCR_SITFACTIVE);

  putreg32(MDF_SITFCR_STH(MDF_SITF_THRESHOLD) | MDF_SITFCR_SITFMOD_SPI |
           MDF_SITFCR_SCKSRC_CCK0, MDF_SITFCR);
  modifyreg32(MDF_SITFCR, 0, MDF_SITFCR_SITFEN);

  /* Filter 0 takes bitstream 0 sampled on the rising edge, where the DK
   * microphone drives its data (the ST BSP selects the falling edge)
   */

  putreg32(MDF_BSMXCR_BSSEL(0, 0), MDF_BSMXCR);

  /* Continuous acquisition with DMA requests while the RXFIFO is not
   * empty, the first samples are discarded.
   */

  putreg32(MDF_DFLTCR_DMAEN | MDF_DFLTCR_ACQMOD_ACONT |
           MDF_DFLTCR_NBDIS(MDF_DISCARD), MDF_DFLTCR);

  /* Sinc4 filter on the bitstream matrix.  A negative scale is encoded
   * with an offset of -16 in the 6-bit field.
   */

  scale = rate->gain >= 0 ? rate->gain : (rate->gain - 16);
  scale = (scale << MDF_DFLTCICR_SCALE_SHIFT) & MDF_DFLTCICR_SCALE_MASK;

  putreg32(MDF_DFLTCICR_DATSRC_BSMX | MDF_DFLTCICR_CICMOD_SINC4 |
           MDF_DFLTCICR_MCICD(rate->mcicd) | scale, MDF_DFLTCICR);

  putreg32(0, MDF_DLYCR);
  putreg32(0, MDF_OECCR);

  /* Reshape filter decimating by 4 and the high-pass filter removing the
   * DC offset (cut-off 0.000625 x Fpcm), no integrator.
   */

  putreg32(MDF_DFLTRSFR_HPFC_0_000625, MDF_DFLTRSFR);
  putreg32(0, MDF_DFLTINTR);

  /* No interrupts, clear the stale status flags */

  putreg32(0, MDF_DFLTIER);
  putreg32(MDF_DFLTINT_ALL, MDF_DFLTISR);
}

/****************************************************************************
 * Name: mdf_hwstop
 *
 * Description:
 *   Stop the filter, the serial interface and the microphone clock.
 *
 ****************************************************************************/

static void mdf_hwstop(void)
{
  modifyreg32(MDF_DFLTCR, MDF_DFLTCR_DFLTEN, 0);
  mdf_waitclear(MDF_DFLTCR, MDF_DFLTCR_DFLTACTIVE);
  putreg32(0, MDF_DFLTCR);

  modifyreg32(MDF_SITFCR, MDF_SITFCR_SITFEN, 0);
  modifyreg32(STM32_MDF1_CKGCR, MDF_CKGCR_CKDEN, 0);
}

/****************************************************************************
 * Name: mdf_dmastart
 *
 * Description:
 *   Start the DMA into the next pending buffer if none is being filled.
 *
 * Assumptions:
 *   Interrupts are disabled.
 *
 ****************************************************************************/

static void mdf_dmastart(FAR struct stm32_mdf_s *priv)
{
  struct stm32_gpdma_cfg_s dmacfg;
  FAR struct ap_buffer_s *apb;
  uintptr_t samp;
  apb_samp_t nbytes;

  if (!priv->running || priv->act != NULL)
    {
      return;
    }

  apb = (FAR struct ap_buffer_s *)dq_remfirst(&priv->pendq);
  if (apb == NULL)
    {
      return;
    }

  samp   = (uintptr_t)apb->samp;
  nbytes = mdf_dmabytes(apb);

  /* The DMA does not maintain the data cache: drop the lines of the
   * buffer so that no dirty line is written back over the samples.
   */

  up_invalidate_dcache(samp, samp + nbytes);

  dmacfg.src_addr   = MDF_DFLTDR;
  dmacfg.dest_addr  = samp;
  dmacfg.tr1        = MDF_DMA_CONFIG;
  dmacfg.request    = GPDMA_REQ_MDF1_FLT0;
  dmacfg.ntransfers = nbytes;
  dmacfg.priority   = GPDMACFG_PRIO_H;
  dmacfg.mode       = 0;

  priv->act = apb;

  stm32_dmasetup(priv->dma, &dmacfg);
  stm32_dmastart(priv->dma, mdf_dmacallback, priv, false);
}

/****************************************************************************
 * Name: mdf_dmacallback
 *
 * Description:
 *   DMA completion: queue the filled buffer for the worker and continue
 *   with the next pending buffer.
 *
 ****************************************************************************/

static void mdf_dmacallback(DMA_HANDLE handle, uint8_t status, void *arg)
{
  FAR struct stm32_mdf_s *priv = (FAR struct stm32_mdf_s *)arg;
  FAR struct ap_buffer_s *apb;
  uintptr_t samp;

  apb = priv->act;
  if (apb == NULL)
    {
      return;
    }

  priv->act = NULL;

  if ((status & DMA_STATUS_FATAL) != 0)
    {
      auderr("ERROR: DMA status %02x\n", status);
    }

  /* Drop any cache lines speculatively loaded during the transfer */

  samp        = (uintptr_t)apb->samp;
  apb->nbytes = mdf_dmabytes(apb);
  up_invalidate_dcache(samp, samp + apb->nbytes);

  dq_addlast(&apb->dq_entry, &priv->doneq);

  /* Keep the RXFIFO drained */

  mdf_dmastart(priv);

  work_queue(HPWORK, &priv->work, mdf_worker, priv, 0);
}

/****************************************************************************
 * Name: mdf_convert
 *
 * Description:
 *   Convert the 24-bit left aligned filter output in place to saturated
 *   16-bit PCM.  Each 16-bit sample (or stereo pair) is written at or
 *   below the 32-bit word it is computed from.
 *
 ****************************************************************************/

static void mdf_convert(FAR struct stm32_mdf_s *priv,
                        FAR struct ap_buffer_s *apb)
{
  FAR const int32_t *src = (FAR const int32_t *)apb->samp;
  FAR int16_t *dest = (FAR int16_t *)apb->samp;
  apb_samp_t nsamples = apb->nbytes / sizeof(int32_t);
  apb_samp_t i;
  int32_t sample;

  for (i = 0; i < nsamples; i++)
    {
      sample = src[i] >> MDF_DFLTDR_DR_SHIFT;
      sample = MAX(MIN(sample, INT16_MAX), INT16_MIN);

      if (priv->mute > 0)
        {
          priv->mute--;
          sample = 0;
        }

      if (priv->nchannels == 2)
        {
          dest[2 * i]     = (int16_t)sample;
          dest[2 * i + 1] = (int16_t)sample;
        }
      else
        {
          dest[i] = (int16_t)sample;
        }
    }

  apb->nbytes   = nsamples * priv->nchannels * sizeof(int16_t);
  apb->nsamples = nsamples;
  apb->curbyte  = 0;
}

/****************************************************************************
 * Name: mdf_worker
 *
 * Description:
 *   Convert and return the filled buffers.  After a stop, return the
 *   buffers that were not filled and report the end of the stream.
 *
 ****************************************************************************/

static void mdf_worker(FAR void *arg)
{
  FAR struct stm32_mdf_s *priv = (FAR struct stm32_mdf_s *)arg;
  FAR struct ap_buffer_s *apb;
  struct dq_queue_s unused;
  irqstate_t flags;
  bool complete;

  for (; ; )
    {
      flags = enter_critical_section();
      apb = (FAR struct ap_buffer_s *)dq_remfirst(&priv->doneq);
      leave_critical_section(flags);

      if (apb == NULL)
        {
          break;
        }

      mdf_convert(priv, apb);
      mdf_callback(priv, AUDIO_CALLBACK_DEQUEUE, apb);
    }

  dq_init(&unused);

  flags = enter_critical_section();
  complete = priv->stopping;
  if (complete)
    {
      dq_move(&priv->pendq, &unused);
      priv->stopping = false;
    }

  leave_critical_section(flags);

  if (complete)
    {
      while ((apb = (FAR struct ap_buffer_s *)dq_remfirst(&unused)) != NULL)
        {
          apb->nbytes = 0;
          mdf_callback(priv, AUDIO_CALLBACK_DEQUEUE, apb);
        }

      mdf_callback(priv, AUDIO_CALLBACK_COMPLETE, NULL);
    }
}

/****************************************************************************
 * Name: mdf_getrate
 *
 * Description:
 *   Look up the filter settings for a sample rate.
 *
 ****************************************************************************/

static FAR const struct mdf_rate_s *mdf_getrate(uint32_t rate)
{
  int i;

  for (i = 0; i < nitems(g_mdf_rates); i++)
    {
      if (g_mdf_rates[i].rate == rate)
        {
          return &g_mdf_rates[i];
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: mdf_getcaps
 ****************************************************************************/

static int mdf_getcaps(FAR struct audio_lowerhalf_s *dev, int type,
                       FAR struct audio_caps_s *caps)
{
  DEBUGASSERT(caps && caps->ac_len >= sizeof(struct audio_caps_s));
  audinfo("type=%d ac_type=%d\n", type, caps->ac_type);

  caps->ac_format.hw  = 0;
  caps->ac_controls.w = 0;

  switch (caps->ac_type)
    {
      case AUDIO_TYPE_QUERY:
        caps->ac_channels = 2;

        if (caps->ac_subtype == AUDIO_TYPE_QUERY)
          {
            /* An input unit producing PCM */

            caps->ac_controls.b[0] = AUDIO_TYPE_INPUT;
            caps->ac_format.hw     = 1 << (AUDIO_FMT_PCM - 1);
          }
        else
          {
            caps->ac_controls.b[0] = AUDIO_SUBFMT_END;
          }
        break;

      case AUDIO_TYPE_INPUT:
        caps->ac_channels = 2;

        if (caps->ac_subtype == AUDIO_TYPE_QUERY)
          {
            caps->ac_controls.hw[0] = AUDIO_SAMP_RATE_8K  |
                                      AUDIO_SAMP_RATE_16K |
                                      AUDIO_SAMP_RATE_32K |
                                      AUDIO_SAMP_RATE_48K;
          }
        break;

      default:
        break;
    }

  return caps->ac_len;
}

/****************************************************************************
 * Name: mdf_configure
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int mdf_configure(FAR struct audio_lowerhalf_s *dev,
                         FAR void *session,
                         FAR const struct audio_caps_s *caps)
#else
static int mdf_configure(FAR struct audio_lowerhalf_s *dev,
                         FAR const struct audio_caps_s *caps)
#endif
{
  FAR struct stm32_mdf_s *priv = (FAR struct stm32_mdf_s *)dev;
  FAR const struct mdf_rate_s *rate;
  uint32_t samprate;

  DEBUGASSERT(priv != NULL && caps != NULL);

  if (caps->ac_type != AUDIO_TYPE_INPUT)
    {
      return OK;
    }

  samprate = caps->ac_controls.hw[0] |
             ((uint32_t)caps->ac_controls.b[3] << 16);

  audinfo("rate %" PRIu32 " channels %u bits %u\n", samprate,
          caps->ac_channels, caps->ac_controls.b[2]);

  rate = mdf_getrate(samprate);
  if (rate == NULL || caps->ac_channels < 1 || caps->ac_channels > 2 ||
      caps->ac_controls.b[2] != 16)
    {
      return -EINVAL;
    }

  if (priv->running)
    {
      return -EBUSY;
    }

  priv->rate      = rate;
  priv->nchannels = caps->ac_channels;
  return OK;
}

/****************************************************************************
 * Name: mdf_shutdown
 ****************************************************************************/

static int mdf_shutdown(FAR struct audio_lowerhalf_s *dev)
{
  FAR struct stm32_mdf_s *priv = (FAR struct stm32_mdf_s *)dev;
  irqstate_t flags;

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
  mdf_stop(dev, NULL);
#else
  mdf_stop(dev);
#endif
#endif

  /* Forget buffers enqueued without a start; their owner frees them */

  flags = enter_critical_section();
  if (!priv->running && !priv->stopping)
    {
      dq_init(&priv->pendq);
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: mdf_start
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int mdf_start(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int mdf_start(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct stm32_mdf_s *priv = (FAR struct stm32_mdf_s *)dev;
  irqstate_t flags;

  if (priv->running || priv->stopping)
    {
      return -EBUSY;
    }

  mdf_hwstart(priv);
  priv->mute = priv->rate->rate * MDF_MUTE_MS / 1000;

  /* Arm the DMA before the filter as the ST HAL does */

  flags = enter_critical_section();
  priv->running = true;
  mdf_dmastart(priv);
  leave_critical_section(flags);

  modifyreg32(MDF_DFLTCR, 0, MDF_DFLTCR_DFLTEN);
  return OK;
}

/****************************************************************************
 * Name: mdf_stop
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int mdf_stop(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int mdf_stop(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct stm32_mdf_s *priv = (FAR struct stm32_mdf_s *)dev;
  irqstate_t flags;

  flags = enter_critical_section();
  if (!priv->running)
    {
      leave_critical_section(flags);
      return OK;
    }

  priv->running = false;
  leave_critical_section(flags);

  mdf_hwstop();
  stm32_dmastop(priv->dma);

  /* The partly filled buffer is returned empty with the pending ones */

  flags = enter_critical_section();
  if (priv->act != NULL)
    {
      dq_addfirst(&priv->act->dq_entry, &priv->pendq);
      priv->act = NULL;
    }

  priv->stopping = true;
  leave_critical_section(flags);

  work_queue(HPWORK, &priv->work, mdf_worker, priv, 0);
  return OK;
}
#endif

/****************************************************************************
 * Name: mdf_pause
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int mdf_pause(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int mdf_pause(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct stm32_mdf_s *priv = (FAR struct stm32_mdf_s *)dev;

  /* Stop the filter, the DMA waits for its requests */

  if (priv->running)
    {
      modifyreg32(MDF_DFLTCR, MDF_DFLTCR_DFLTEN, 0);
      mdf_waitclear(MDF_DFLTCR, MDF_DFLTCR_DFLTACTIVE);
    }

  return OK;
}

/****************************************************************************
 * Name: mdf_resume
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int mdf_resume(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int mdf_resume(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct stm32_mdf_s *priv = (FAR struct stm32_mdf_s *)dev;

  if (priv->running)
    {
      modifyreg32(MDF_DFLTCR, 0, MDF_DFLTCR_DFLTEN);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: mdf_allocbuffer
 *
 * Description:
 *   Allocate an audio buffer with cache line aligned samples.
 *
 ****************************************************************************/

static int mdf_allocbuffer(FAR struct audio_lowerhalf_s *dev,
                           FAR struct audio_buf_desc_s *bufdesc)
{
  FAR struct stm32_mdf_s *priv = (FAR struct stm32_mdf_s *)dev;
  FAR struct ap_buffer_s *apb;

  DEBUGASSERT(bufdesc->u.pbuffer != NULL);

  if (bufdesc->numbytes < sizeof(uint32_t))
    {
      return -EINVAL;
    }

  apb = kumm_zalloc(sizeof(struct ap_buffer_s));
  if (apb == NULL)
    {
      return -ENOMEM;
    }

  apb->samp = kumm_memalign(MDF_BUFALIGN,
                            ALIGN_UP(bufdesc->numbytes, MDF_BUFALIGN));
  if (apb->samp == NULL)
    {
      kumm_free(apb);
      return -ENOMEM;
    }

  apb->i.channels = priv->nchannels;
  apb->crefs      = 1;
  apb->nmaxbytes  = bufdesc->numbytes;
#ifdef CONFIG_AUDIO_MULTI_SESSION
  apb->session    = bufdesc->session;
#endif
  nxmutex_init(&apb->lock);

  *bufdesc->u.pbuffer = apb;
  return sizeof(struct audio_buf_desc_s);
}

/****************************************************************************
 * Name: mdf_freebuffer
 ****************************************************************************/

static int mdf_freebuffer(FAR struct audio_lowerhalf_s *dev,
                          FAR struct audio_buf_desc_s *bufdesc)
{
  FAR struct ap_buffer_s *apb = bufdesc->u.buffer;

  DEBUGASSERT(apb != NULL);

  nxmutex_destroy(&apb->lock);
  kumm_free(apb->samp);
  kumm_free(apb);
  return sizeof(struct audio_buf_desc_s);
}

/****************************************************************************
 * Name: mdf_enqueuebuffer
 ****************************************************************************/

static int mdf_enqueuebuffer(FAR struct audio_lowerhalf_s *dev,
                             FAR struct ap_buffer_s *apb)
{
  FAR struct stm32_mdf_s *priv = (FAR struct stm32_mdf_s *)dev;
  irqstate_t flags;

  if (apb->nmaxbytes < sizeof(uint32_t))
    {
      return -EINVAL;
    }

  apb->nbytes  = 0;
  apb->curbyte = 0;

  flags = enter_critical_section();
  dq_addlast(&apb->dq_entry, &priv->pendq);
  mdf_dmastart(priv);
  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: mdf_ioctl
 ****************************************************************************/

static int mdf_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd,
                     unsigned long arg)
{
  FAR struct ap_buffer_info_s *bufinfo;

  switch (cmd)
    {
      /* Report the preferred buffer size and quantity */

      case AUDIOIOC_GETBUFFERINFO:
        bufinfo              = (FAR struct ap_buffer_info_s *)arg;
        bufinfo->buffer_size = MIN(CONFIG_AUDIO_BUFFER_NUMBYTES & ~3,
                                   MDF_DMA_MAXBYTES);
        bufinfo->nbuffers    = CONFIG_AUDIO_NUM_BUFFERS;
        return OK;

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Name: mdf_reserve
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int mdf_reserve(FAR struct audio_lowerhalf_s *dev,
                       FAR void **session)
#else
static int mdf_reserve(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct stm32_mdf_s *priv = (FAR struct stm32_mdf_s *)dev;
  irqstate_t flags;
  int ret = OK;

  flags = enter_critical_section();
  if (priv->reserved)
    {
      ret = -EBUSY;
    }
  else
    {
#ifdef CONFIG_AUDIO_MULTI_SESSION
      *session       = NULL;
#endif
      priv->reserved = true;
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: mdf_release
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int mdf_release(FAR struct audio_lowerhalf_s *dev,
                       FAR void *session)
#else
static int mdf_release(FAR struct audio_lowerhalf_s *dev)
#endif
{
  FAR struct stm32_mdf_s *priv = (FAR struct stm32_mdf_s *)dev;

  priv->reserved = false;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_mdf_initialize
 *
 * Description:
 *   Initialize MDF1 filter 0 for a PDM microphone and return an audio
 *   capture lower half.
 *
 ****************************************************************************/

FAR struct audio_lowerhalf_s *stm32_mdf_initialize(void)
{
  FAR struct stm32_mdf_s *priv = &g_mdf1;

  if (priv->dma != NULL)
    {
      return &priv->dev;
    }

  priv->dma = stm32_dmachannel(GPDMA_TTYPE_P2M);
  if (priv->dma == NULL)
    {
      auderr("ERROR: No DMA channel\n");
      return NULL;
    }

  stm32_configgpio(GPIO_MDF1_CCK0);
  stm32_configgpio(GPIO_MDF1_SDI0);

  dq_init(&priv->pendq);
  dq_init(&priv->doneq);
  priv->rate      = mdf_getrate(MDF_DEFAULT_RATE);
  priv->nchannels = 1;

  return &priv->dev;
}

#endif /* CONFIG_STM32_MDF1 */
