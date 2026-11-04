/****************************************************************************
 * arch/arm/src/common/stm32/stm32_sai.c
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 2019 Gregory Nutt. All rights reserved.
 * SPDX-FileCopyrightText: 2016 Motorola Mobility LLC. All rights reserved.
 * SPDX-FileCopyrightText: 2013-2014 Gregory Nutt. All rights reserved.
 * SPDX-FileContributor: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <nuttx/debug.h>
#include <nuttx/nuttx.h>

#include <arch/board/board.h>

#include <nuttx/cache.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wdog.h>
#include <nuttx/irq.h>
#include <nuttx/queue.h>
#include <nuttx/wqueue.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/i2s.h>

#include "stm32.h"
#include "arm_internal.h"
#include "stm32_sai.h"

#ifdef CONFIG_STM32_SAI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SCHED_WORKQUEUE
#  error Work queue support is required (CONFIG_SCHED_WORKQUEUE)
#endif

#ifndef CONFIG_AUDIO
#  error CONFIG_AUDIO required by this driver
#endif

#ifndef CONFIG_I2S
#  error CONFIG_I2S required by this driver
#endif

#ifdef CONFIG_STM32_SAI_POLLING
#  error "Polling SAI not yet supported"
#endif

#ifdef CONFIG_STM32_SAI_INTERRUPTS
#  error "Interrupt driven SAI not yet supported"
#endif

#ifndef CONFIG_STM32_SAI_DEFAULT_SAMPLERATE
#  define CONFIG_STM32_SAI_DEFAULT_SAMPLERATE  (48000)
#endif

#ifndef CONFIG_STM32_SAI_DEFAULT_DATALEN
#  define CONFIG_STM32_SAI_DEFAULT_DATALEN     (16)
#endif

#ifndef CONFIG_STM32_SAI_MAXINFLIGHT
#  define CONFIG_STM32_SAI_MAXINFLIGHT         (16)
#endif

/* After SAIEN is cleared, the audio block stops at the end of the current
 * frame.  Wait for up to two frames.
 */

#define SAI_DISABLE_TIMEOUT_US(rate)           (2 * USEC_PER_SEC / (rate) + 1)

#ifdef CONFIG_STM32_SAI1
#ifndef STM32_SAI1_FREQUENCY
#  error "Please define STM32_SAI1_FREQUENCY in board.h"
#endif
#endif

#ifdef CONFIG_STM32_SAI2
#ifndef STM32_SAI2_FREQUENCY
#  error "Please define STM32_SAI1_FREQUENCY in board.h"
#endif
#endif

#if defined(CONFIG_STM32_SAI_DMA) && defined(CONFIG_STM32_HAVE_IP_DMA_GPDMA)
/* GPDMA: the channel is selected by the request line, the transfer
 * register 1 holds the data widths and the incrementing side.
 */

#  define DMACHAN_SAI1_A       GPDMA_REQ_SAI1_A
#  define DMACHAN_SAI1_B       GPDMA_REQ_SAI1_B
#  define DMACHAN_SAI2_A       GPDMA_REQ_SAI2_A
#  define DMACHAN_SAI2_B       GPDMA_REQ_SAI2_B

#  define SAI_DMA_PRIO         GPDMACFG_PRIO_H
#  define SAI_DMA_ERROR        DMA_STATUS_FATAL

#  define SAI_RXDMA8_CONFIG    (GPDMA_CXTR1_SDW_LOG2_BYTE | \
                                GPDMA_CXTR1_DDW_LOG2_BYTE | GPDMA_CXTR1_DINC)
#  define SAI_RXDMA16_CONFIG   (GPDMA_CXTR1_SDW_LOG2_HW | \
                                GPDMA_CXTR1_DDW_LOG2_HW | GPDMA_CXTR1_DINC)
#  define SAI_RXDMA32_CONFIG   (GPDMA_CXTR1_SDW_LOG2_WORD | \
                                GPDMA_CXTR1_DDW_LOG2_WORD | GPDMA_CXTR1_DINC)

#  define SAI_TXDMA8_CONFIG    (GPDMA_CXTR1_SDW_LOG2_BYTE | \
                                GPDMA_CXTR1_DDW_LOG2_BYTE | GPDMA_CXTR1_SINC)
#  define SAI_TXDMA16_CONFIG   (GPDMA_CXTR1_SDW_LOG2_HW | \
                                GPDMA_CXTR1_DDW_LOG2_HW | GPDMA_CXTR1_SINC)
#  define SAI_TXDMA32_CONFIG   (GPDMA_CXTR1_SDW_LOG2_WORD | \
                                GPDMA_CXTR1_DDW_LOG2_WORD | GPDMA_CXTR1_SINC)

#elif defined(CONFIG_STM32_SAI_DMA)
/* SAI DMA priority */

#  define SAI_DMA_ERROR        DMA_STREAM_TEIF_BIT

#  if defined(CONFIG_STM32_SAI_DMAPRIO)
#    define SAI_DMA_PRIO       CONFIG_STM32_SAI_DMAPRIO
#  else
#    define SAI_DMA_PRIO       DMA_SCR_PRIVERYHI
#  endif

#  if (SAI_DMA_PRIO & ~DMA_SCR_PL_MASK) != 0
#    error "Illegal value for CONFIG_STM32_SAI_DMAPRIO"
#  endif

/* DMA channel/stream configuration register settings.  The following
 * must be selected.  The DMA driver will select the remaining fields.
 *
 * - 32-bit DMA
 * - Memory increment
 * - Direction (memory-to-peripheral, peripheral-to-memory)
 * - Memory burst size (F4 only)
 */

/* STM32 stream configuration register (SCR) settings base settings sans
 * priority.
 */

#  define SAI_RXDMA8_CONFIG   (DMA_SCR_PFCTRL | DMA_SCR_DIR_P2M|DMA_SCR_MINC | \
                                  DMA_SCR_PSIZE_8BITS | DMA_SCR_MSIZE_8BITS | \
                                  DMA_SCR_PBURST_INCR4 | DMA_SCR_MBURST_INCR4)
#  define SAI_RXDMA16_CONFIG   (DMA_SCR_PFCTRL | DMA_SCR_DIR_P2M|DMA_SCR_MINC | \
                                  DMA_SCR_PSIZE_16BITS | DMA_SCR_MSIZE_16BITS | \
                                  DMA_SCR_PBURST_INCR4 | DMA_SCR_MBURST_INCR4)
#  define SAI_RXDMA32_CONFIG   (DMA_SCR_PFCTRL | DMA_SCR_DIR_P2M|DMA_SCR_MINC | \
                                  DMA_SCR_PSIZE_32BITS | DMA_SCR_MSIZE_32BITS | \
                                  DMA_SCR_PBURST_INCR4 | DMA_SCR_MBURST_INCR4)

#  define SAI_TXDMA8_CONFIG    (DMA_SCR_DIR_M2P | DMA_SCR_MINC | \
                                  DMA_SCR_PSIZE_8BITS | DMA_SCR_MSIZE_8BITS | \
                                  DMA_SCR_PBURST_INCR4 | DMA_SCR_MBURST_INCR4)
#  define SAI_TXDMA16_CONFIG   (DMA_SCR_DIR_M2P | DMA_SCR_MINC | \
                                  DMA_SCR_PSIZE_16BITS | DMA_SCR_MSIZE_16BITS | \
                                  DMA_SCR_PBURST_INCR4 | DMA_SCR_MBURST_INCR4)
#  define SAI_TXDMA32_CONFIG   (DMA_SCR_DIR_M2P | DMA_SCR_MINC | \
                                  DMA_SCR_PSIZE_32BITS | DMA_SCR_MSIZE_32BITS | \
                                  DMA_SCR_PBURST_INCR4 | DMA_SCR_MBURST_INCR4)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* I2S buffer container */

struct sai_buffer_s
{
  struct sai_buffer_s *flink;  /* Supports a singly linked list */
  i2s_callback_t callback;     /* Function to call when the transfer completes */
  uint32_t timeout;            /* The timeout value to use with transfers */
  void *arg;                   /* The argument to be returned with the callback */
  struct ap_buffer_s *apb;     /* The audio buffer */
  uint8_t *rxbuf;              /* Receive DMA buffer, NULL when sending */
  int result;                  /* The result of the transfer */
};

/* The state of the one SAI peripheral */

struct stm32_sai_s
{
  struct i2s_dev_s dev;        /* Externally visible I2S interface */

  /* Callback for changes in sample rate */

  stm32_sai_sampleratecb_t sampleratecb;
  uintptr_t base;              /* SAI block register base address */
  mutex_t lock;                /* Assures mutually exclusive access to SAI */
  uint32_t frequency;          /* SAI clock frequency */
  uint32_t syncen;             /* Synchronization setting */
#ifdef CONFIG_STM32_SAI_DMA
  uint16_t dma_ch;             /* DMA channel number or request */
  DMA_HANDLE dma;              /* DMA channel handle */
  uint32_t dma_ccr;            /* DMA control or transfer register */
#endif
  uint8_t datalen;             /* Data width */
  uint32_t samplerate;         /* Data sample rate */
  uint8_t rxenab:1;            /* True: RX transfers enabled */
  uint8_t txenab:1;            /* True: TX transfers enabled */
  struct wdog_s dog;           /* Watchdog that handles timeouts */
  sq_queue_t pend;             /* A queue of pending transfers */
  sq_queue_t act;              /* A queue of active transfers */
  sq_queue_t done;             /* A queue of completed transfers */
  struct work_s work;          /* Supports worker thread operations */

  /* Pre-allocated pool of buffer containers */

  sem_t bufsem;                   /* Buffer wait semaphore */
  struct sai_buffer_s *freelist;  /* A list a free buffer containers */
  struct sai_buffer_s containers[CONFIG_STM32_SAI_MAXINFLIGHT];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_DEBUG_I2S_INFO
static void     sai_dump_regs(struct stm32_sai_s *priv, const char *msg);
#else
#  define       sai_dump_regs(s,m)
#endif

/* Buffer container helpers */

static struct sai_buffer_s *
                sai_buf_allocate(struct stm32_sai_s *priv);
static void     sai_buf_free(struct stm32_sai_s *priv,
                  struct sai_buffer_s *bfcontainer);
static void     sai_buf_initialize(struct stm32_sai_s *priv);

/* DMA support */

#ifdef CONFIG_STM32_SAI_DMA
static void     sai_schedule(struct stm32_sai_s *priv, int result);
static void     sai_dma_callback(DMA_HANDLE handle, uint8_t isr, void *arg);
#endif

/* I2S methods */

static int32_t  sai_samplerate(struct i2s_dev_s *dev, uint32_t rate);
static int32_t  sai_datawidth(struct i2s_dev_s *dev, int bits);
static int      sai_receive(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                  i2s_callback_t callback, void *arg, uint32_t timeout);
static int      sai_send(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                  i2s_callback_t callback, void *arg,
                  uint32_t timeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I2S device operations */

static const struct i2s_ops_s g_i2sops =
{
  /* Receiver methods */

  .i2s_rxsamplerate = sai_samplerate,
  .i2s_rxdatawidth  = sai_datawidth,
  .i2s_receive      = sai_receive,

  /* Transmitter methods */

  .i2s_txsamplerate = sai_samplerate,
  .i2s_txdatawidth  = sai_datawidth,
  .i2s_send         = sai_send,
};

/* SAI1 state */

#ifdef CONFIG_STM32_SAI1_A
static struct stm32_sai_s g_sai1a_priv =
{
  .dev.ops     = &g_i2sops,
  .base        = STM32_SAI1_A_BASE,
  .lock        = NXMUTEX_INITIALIZER,
  .frequency   = STM32_SAI1_FREQUENCY,
#ifdef CONFIG_STM32_SAI1_A_SYNC_WITH_B
  .syncen      = SAI_CR1_SYNCEN_INTERNAL,
#else
  .syncen      = SAI_CR1_SYNCEN_ASYNCH,
#endif
#ifdef CONFIG_STM32_SAI_DMA
  .dma_ch      = DMACHAN_SAI1_A,
#endif
  .datalen     = CONFIG_STM32_SAI_DEFAULT_DATALEN,
  .samplerate  = CONFIG_STM32_SAI_DEFAULT_SAMPLERATE,
  .bufsem      = SEM_INITIALIZER(CONFIG_STM32_SAI_MAXINFLIGHT),
};
#endif

#ifdef CONFIG_STM32_SAI1_B
static struct stm32_sai_s g_sai1b_priv =
{
  .dev.ops     = &g_i2sops,
  .base        = STM32_SAI1_B_BASE,
  .lock        = NXMUTEX_INITIALIZER,
  .frequency   = STM32_SAI1_FREQUENCY,
#ifdef CONFIG_STM32_SAI1_B_SYNC_WITH_A
  .syncen      = SAI_CR1_SYNCEN_INTERNAL,
#else
  .syncen      = SAI_CR1_SYNCEN_ASYNCH,
#endif
#ifdef CONFIG_STM32_SAI_DMA
  .dma_ch      = DMACHAN_SAI1_B,
#endif
  .datalen     = CONFIG_STM32_SAI_DEFAULT_DATALEN,
  .samplerate  = CONFIG_STM32_SAI_DEFAULT_SAMPLERATE,
  .bufsem      = SEM_INITIALIZER(CONFIG_STM32_SAI_MAXINFLIGHT),
};
#endif

/* SAI2 state */

#ifdef CONFIG_STM32_SAI2_A
static struct stm32_sai_s g_sai2a_priv =
{
  .dev.ops     = &g_i2sops,
  .base        = STM32_SAI2_A_BASE,
  .lock        = NXMUTEX_INITIALIZER,
  .frequency   = STM32_SAI2_FREQUENCY,
#ifdef CONFIG_STM32_SAI2_A_SYNC_WITH_B
  .syncen      = SAI_CR1_SYNCEN_INTERNAL,
#else
  .syncen      = SAI_CR1_SYNCEN_ASYNCH,
#endif
#ifdef CONFIG_STM32_SAI_DMA
  .dma_ch      = DMACHAN_SAI2_A,
#endif
  .datalen     = CONFIG_STM32_SAI_DEFAULT_DATALEN,
  .samplerate  = CONFIG_STM32_SAI_DEFAULT_SAMPLERATE,
  .bufsem      = SEM_INITIALIZER(CONFIG_STM32_SAI_MAXINFLIGHT),
};
#endif

#ifdef CONFIG_STM32_SAI2_B
static struct stm32_sai_s g_sai2b_priv =
{
  .dev.ops     = &g_i2sops,
  .base        = STM32_SAI2_B_BASE,
  .lock        = NXMUTEX_INITIALIZER,
  .frequency   = STM32_SAI2_FREQUENCY,
#ifdef CONFIG_STM32_SAI2_B_SYNC_WITH_A
  .syncen      = SAI_CR1_SYNCEN_INTERNAL,
#else
  .syncen      = SAI_CR1_SYNCEN_ASYNCH,
#endif
#ifdef CONFIG_STM32_SAI_DMA
  .dma_ch      = DMACHAN_SAI2_B,
#endif
  .datalen     = CONFIG_STM32_SAI_DEFAULT_DATALEN,
  .samplerate  = CONFIG_STM32_SAI_DEFAULT_SAMPLERATE,
  .bufsem      = SEM_INITIALIZER(CONFIG_STM32_SAI_MAXINFLIGHT),
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sai_getbitrate
 *
 * Description:
 *   Get the currently configured bitrate
 *
 * Input Parameters:
 *   priv   - private SAI device structure
 *
 * Returned Value:
 *   The current bitrate
 *
 ****************************************************************************/

static inline uint32_t sai_getbitrate(struct stm32_sai_s *priv)
{
  /* Calculate the bitrate in Hz */

  return priv->samplerate * priv->datalen;
}

/****************************************************************************
 * Name: sai_getreg
 *
 * Description:
 *   Get the contents of the SAI register at offset
 *
 * Input Parameters:
 *   priv   - private SAI device structure
 *   offset - offset to the register of interest
 *
 * Returned Value:
 *   The contents of the 32-bit register
 *
 ****************************************************************************/

static inline uint32_t sai_getreg(struct stm32_sai_s *priv, uint8_t offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: sai_putreg
 *
 * Description:
 *   Write a 16-bit value to the SAI register at offset
 *
 * Input Parameters:
 *   priv   - private SAI device structure
 *   offset - offset to the register of interest
 *   value  - the 32-bit value to be written
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void sai_putreg(struct stm32_sai_s *priv, uint8_t offset,
                              uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: sai_modifyreg
 *
 * Description:
 *   Clear and set bits in the SAI register at offset
 *
 * Input Parameters:
 *   priv    - private SAI device structure
 *   offset  - offset to the register of interest
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sai_modifyreg(struct stm32_sai_s *priv, uint8_t offset,
                          uint32_t clrbits, uint32_t setbits)
{
  uint32_t regval;

  regval  = sai_getreg(priv, offset);
  regval &= ~clrbits;
  regval |= setbits;
  sai_putreg(priv, offset, regval);
}

/****************************************************************************
 * Name: sai_disable
 *
 * Description:
 *   Disable the audio block so that its configuration may be changed and
 *   wait until it has stopped at the end of the current frame.
 *
 * Input Parameters:
 *   priv    - private SAI device structure
 *   enabled - Returned true if the audio block was enabled
 *
 * Returned Value:
 *   OK on success; -ETIMEDOUT if the audio block did not stop
 *
 ****************************************************************************/

static int sai_disable(struct stm32_sai_s *priv, bool *enabled)
{
  uint32_t timeout;

  DEBUGASSERT(priv->samplerate > 0);

  *enabled = (sai_getreg(priv, STM32_SAI_CR1_OFFSET) & SAI_CR1_SAIEN) != 0;
  if (!*enabled)
    {
      return OK;
    }

  sai_modifyreg(priv, STM32_SAI_CR1_OFFSET, SAI_CR1_SAIEN, 0);

  for (timeout = SAI_DISABLE_TIMEOUT_US(priv->samplerate); timeout > 0;
       timeout--)
    {
      if ((sai_getreg(priv, STM32_SAI_CR1_OFFSET) & SAI_CR1_SAIEN) == 0)
        {
          return OK;
        }

      up_udelay(1);
    }

  i2serr("ERROR: The audio block did not stop\n");
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: sai_enable
 *
 * Description:
 *   Re-enable the audio block if it was enabled before sai_disable().
 *
 * Input Parameters:
 *   priv    - private SAI device structure
 *   enabled - value returned by sai_disable()
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sai_enable(struct stm32_sai_s *priv, bool enabled)
{
  if (enabled)
    {
      sai_modifyreg(priv, STM32_SAI_CR1_OFFSET, 0, SAI_CR1_SAIEN);
    }
}

/****************************************************************************
 * Name: sai_dump_regs
 *
 * Description:
 *   Dump the contents of all SAI block registers
 *
 * Input Parameters:
 *   priv - The SAI block controller to dump
 *   msg - Message to print before the register data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_I2S_INFO
static void sai_dump_regs(struct stm32_sai_s *priv, const char *msg)
{
  if (msg)
    {
      i2sinfo("%s\n", msg);
    }

  i2sinfo("CR1:%08" PRIx32 " CR2:%08" PRIx32
          "  FRCR:%08" PRIx32 " SLOTR:%08" PRIx32 "\n",
          sai_getreg(priv, STM32_SAI_CR1_OFFSET),
          sai_getreg(priv, STM32_SAI_CR2_OFFSET),
          sai_getreg(priv, STM32_SAI_FRCR_OFFSET),
          sai_getreg(priv, STM32_SAI_SLOTR_OFFSET));
  i2sinfo(" IM:%08" PRIx32 "  SR:%08" PRIx32
          " CLRFR:%08" PRIx32 "\n",
          sai_getreg(priv, STM32_SAI_IM_OFFSET),
          sai_getreg(priv, STM32_SAI_SR_OFFSET),
          sai_getreg(priv, STM32_SAI_CLRFR_OFFSET));
}
#endif

/****************************************************************************
 * Name: sai_mckdivider
 *
 * Description:
 *   Setup the master clock divider based on the currently selected data
 *   width and the sample rate
 *
 * Input Parameters:
 *   priv - SAI device structure (only the sample rate and frequency are
 *          needed at this point).
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sai_mckdivider(struct stm32_sai_s *priv)
{
  uint32_t mckdiv;
  uint32_t tmpval;

  DEBUGASSERT(priv && priv->samplerate > 0 && priv->frequency > 0);

#ifdef CONFIG_STM32_HAVE_IP_SAI_V2
  /* MCLK_x = SAI_CK_x / MCKDIV[5:0] (0 and 1 do not divide) with
   * MCLK_x = 256 * FS, rounded to the nearest divider.
   */

  tmpval = priv->samplerate * 256;
  mckdiv = (priv->frequency + tmpval / 2) / tmpval;
  if (mckdiv > 63)
    {
      mckdiv = 63;
    }
#else
  /* Configure Master Clock using the following formula:
   * MCLK_x = SAI_CK_x / (MCKDIV[3:0] * 2) with MCLK_x = 256 * FS
   * FS = SAI_CK_x / (MCKDIV[3:0] * 2) * 256
   * MCKDIV[3:0] = SAI_CK_x / FS * 512
   */

  /* (freq*10) to keep Significant digits */

  tmpval = (priv->frequency * 10) / (priv->samplerate * 2 * 256);
  mckdiv = tmpval / 10;

  /* Round result to the nearest integer */

  if ((tmpval % 10) > 8)
    {
      mckdiv += 1;
    }
#endif

  sai_modifyreg(priv, STM32_SAI_CR1_OFFSET, SAI_CR1_MCKDIV_MASK,
                mckdiv << SAI_CR1_MCKDIV_SHIFT);
}

/****************************************************************************
 * Name: sai_timeout
 *
 * Description:
 *   The watchdog timeout without completion of the transfer.
 *
 * Input Parameters:
 *   arg    - The argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Always called from the interrupt level with interrupts disabled.
 *
 ****************************************************************************/

static void sai_timeout(wdparm_t arg)
{
  struct stm32_sai_s *priv = (struct stm32_sai_s *)arg;

  DEBUGASSERT(priv != NULL);

#ifdef CONFIG_STM32_SAI_DMA
  /* Cancel the DMA */

  stm32_dmastop(priv->dma);
#endif

  /* Then schedule completion of the transfer to occur on the worker
   * thread.
   */

  sai_schedule(priv, -ETIMEDOUT);
}

/****************************************************************************
 * Name: sai_dma_setup
 *
 * Description:
 *   Setup and initiate the next DMA transfer
 *
 * Input Parameters:
 *   priv - SAI state instance
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 * Assumptions:
 *   Interrupts are disabled
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_SAI_DMA
static int sai_dma_setup(struct stm32_sai_s *priv)
{
  struct sai_buffer_s *bfcontainer;
  struct ap_buffer_s *apb;
#ifdef CONFIG_STM32_HAVE_IP_DMA_GPDMA
  struct stm32_gpdma_cfg_s dmacfg;
#endif
  uintptr_t samp;
  apb_samp_t nbytes = 0;
  size_t ntransfers = 0;
  int ret;

  /* If there is already an active transmission in progress, then bail
   * returning success.
   */

  if (!sq_empty(&priv->act))
    {
      return OK;
    }

  /* If there are no pending transfer, then bail returning success */

  if (sq_empty(&priv->pend))
    {
      priv->txenab = priv->rxenab = false;
      return OK;
    }

  /* Remove the pending transfer at the head of the pending queue. */

  bfcontainer = (struct sai_buffer_s *)sq_remfirst(&priv->pend);
  DEBUGASSERT(bfcontainer && bfcontainer->apb);

  apb = bfcontainer->apb;

  /* Get the transfer information, accounting for any data offset */

  samp = (uintptr_t)&apb->samp[apb->curbyte];

  /* Configure the DMA */

  if (priv->txenab)
    {
      nbytes = apb->nbytes - apb->curbyte;

      switch (priv->datalen)
        {
          case 8:
            priv->dma_ccr = SAI_TXDMA8_CONFIG;
            ntransfers = nbytes;
            break;

          case 16:
            priv->dma_ccr = SAI_TXDMA16_CONFIG;
            DEBUGASSERT((nbytes & 0x1) == 0);
            ntransfers = nbytes >> 1;
            break;

          case 32:
            priv->dma_ccr = SAI_TXDMA32_CONFIG;
            DEBUGASSERT((nbytes & 0x3) == 0);
            ntransfers = nbytes >> 2;
            break;
        }
    }
  else if (priv->rxenab)
    {
      nbytes = apb->nmaxbytes - apb->curbyte;

      switch (priv->datalen)
        {
          case 8:
            priv->dma_ccr = SAI_RXDMA8_CONFIG;
            ntransfers = nbytes;
            break;

          case 16:
            priv->dma_ccr = SAI_RXDMA16_CONFIG;
            DEBUGASSERT((nbytes & 0x1) == 0);
            ntransfers = nbytes >> 1;
            break;

          case 32:
            priv->dma_ccr = SAI_RXDMA32_CONFIG;
            DEBUGASSERT((nbytes & 0x3) == 0);
            ntransfers = nbytes >> 2;
            break;
        }
    }

  DEBUGASSERT(ntransfers > 0);

  /* The data cache was maintained when the buffer was queued */

  if (priv->rxenab)
    {
      samp = (uintptr_t)bfcontainer->rxbuf;
    }

#ifdef CONFIG_STM32_HAVE_IP_DMA_GPDMA
  /* The GPDMA block size is in bytes and limited to 16 bits.  The SAI
   * drives the request on the destination side when transmitting.
   */

  DEBUGASSERT(nbytes <= UINT16_MAX);

  if (priv->txenab)
    {
      dmacfg.src_addr  = samp;
      dmacfg.dest_addr = priv->base + STM32_SAI_DR_OFFSET;
      dmacfg.request   = priv->dma_ch | GPDMA_CXTR2_DREQ;
    }
  else
    {
      dmacfg.src_addr  = priv->base + STM32_SAI_DR_OFFSET;
      dmacfg.dest_addr = samp;
      dmacfg.request   = priv->dma_ch;
    }

  dmacfg.tr1        = priv->dma_ccr;
  dmacfg.ntransfers = nbytes;
  dmacfg.priority   = SAI_DMA_PRIO;
  dmacfg.mode       = 0;

  stm32_dmasetup(priv->dma, &dmacfg);
#else
  stm32_dmasetup(priv->dma, priv->base + STM32_SAI_DR_OFFSET,
                 samp, ntransfers, priv->dma_ccr);
#endif

  /* Add the container to the list of active DMAs */

  sq_addlast((sq_entry_t *)bfcontainer, &priv->act);

  /* Start the DMA, saving the container as the current active transfer */

  stm32_dmastart(priv->dma, sai_dma_callback, priv, false);

  /* Enable the transmitter */

  sai_modifyreg(priv, STM32_SAI_CR1_OFFSET, 0, SAI_CR1_SAIEN);

  /* Start a watchdog to catch DMA timeouts */

  if (bfcontainer->timeout > 0)
    {
      ret = wd_start(&priv->dog, bfcontainer->timeout,
                     sai_timeout, (wdparm_t)priv);

      /* Check if we have successfully started the watchdog timer.  Note
       * that we do nothing in the case of failure to start the timer.  We
       * are already committed to the DMA anyway.  Let's just hope that the
       * DMA does not hang.
       */

      if (ret < 0)
        {
          i2serr("ERROR: wd_start failed: %d\n", ret);
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: sai_rxbuf_prepare
 *
 * Description:
 *   Select the receive DMA buffer and maintain the data cache before the
 *   transfer.  The DMA does not snoop the data cache, so the cache lines of
 *   the receive buffer are invalidated before and after the transfer.  An
 *   audio buffer that does not cover whole cache lines shares them with
 *   other data, as its own header: it is received into a cache line
 *   aligned buffer and copied by sai_rxbuf_complete().
 *
 * Input Parameters:
 *   bfcontainer - The buffer container of the transfer
 *   apb         - The audio buffer to receive into
 *
 * Returned Value:
 *   OK on success; -ENOMEM if no aligned buffer could be allocated
 *
 ****************************************************************************/

static int sai_rxbuf_prepare(struct sai_buffer_s *bfcontainer,
                             struct ap_buffer_s *apb)
{
  uint8_t *samp = &apb->samp[apb->curbyte];
  size_t nbytes = apb->nmaxbytes - apb->curbyte;
  size_t linesize = up_get_dcache_linesize();

  bfcontainer->rxbuf = samp;

  if (linesize > 0 &&
      ((((uintptr_t)samp) | nbytes) & (linesize - 1)) != 0)
    {
      bfcontainer->rxbuf = kmm_memalign(linesize,
                                        ALIGN_UP(nbytes, linesize));
      if (bfcontainer->rxbuf == NULL)
        {
          return -ENOMEM;
        }
    }

  up_invalidate_dcache((uintptr_t)bfcontainer->rxbuf,
                       (uintptr_t)bfcontainer->rxbuf + nbytes);
  return OK;
}

/****************************************************************************
 * Name: sai_rxbuf_complete
 *
 * Description:
 *   Drop the cache lines speculatively loaded while the DMA was receiving
 *   and copy the data from the aligned receive buffer if one was used.
 *
 * Input Parameters:
 *   bfcontainer - The buffer container of the completed transfer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sai_rxbuf_complete(struct sai_buffer_s *bfcontainer)
{
  struct ap_buffer_s *apb = bfcontainer->apb;
  uint8_t *samp = &apb->samp[apb->curbyte];
  size_t nbytes = apb->nmaxbytes - apb->curbyte;

  up_invalidate_dcache((uintptr_t)bfcontainer->rxbuf,
                       (uintptr_t)bfcontainer->rxbuf + nbytes);

  if (bfcontainer->rxbuf != samp)
    {
      if (bfcontainer->result == OK)
        {
          memcpy(samp, bfcontainer->rxbuf, nbytes);
        }

      kmm_free(bfcontainer->rxbuf);
    }

  bfcontainer->rxbuf = NULL;
}

/****************************************************************************
 * Name: sai_worker
 *
 * Description:
 *   Transfer done worker
 *
 * Input Parameters:
 *   arg - the SAI device instance cast to void*
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sai_worker(void *arg)
{
  struct stm32_sai_s *priv = (struct stm32_sai_s *)arg;
  struct sai_buffer_s *bfcontainer;
  irqstate_t flags;

  DEBUGASSERT(priv);

  /* When the transfer was started, the active buffer containers were removed
   * from the pend queue and saved in the act queue.  We get here when the
   * transfer is finished... either successfully, with an error, or with a
   * timeout.
   *
   * In any case, the buffer containers in act will be moved to the end
   * of the done queue and act will be emptied before this worker is
   * started.
   */

  i2sinfo("act.head=%p done.head=%p\n", priv->act.head, priv->done.head);

  /* Check if IDLE */

  if (sq_empty(&priv->act))
    {
      /* Then start the next transfer.  This must be done with interrupts
       * disabled.
       */

      flags = enter_critical_section();
#ifdef CONFIG_STM32_SAI_DMA
      sai_dma_setup(priv);
#endif
      leave_critical_section(flags);
    }

  /* Process each buffer in the done queue */

  while (sq_peek(&priv->done) != NULL)
    {
      /* Remove the buffer container from the done queue.  NOTE that
       * interrupts must be enabled to do this because the done queue is
       * also modified from the interrupt level.
       */

      flags = enter_critical_section();
      bfcontainer = (struct sai_buffer_s *)sq_remfirst(&priv->done);
      leave_critical_section(flags);

      /* Make the received data visible to the CPU */

      DEBUGASSERT(bfcontainer && bfcontainer->callback);
      if (bfcontainer->rxbuf != NULL)
        {
          sai_rxbuf_complete(bfcontainer);
        }

      /* Perform the transfer done callback */

      bfcontainer->callback(&priv->dev, bfcontainer->apb,
                            bfcontainer->arg, bfcontainer->result);

      /* Release our reference on the audio buffer.  This may very likely
       * cause the audio buffer to be freed.
       */

      apb_free(bfcontainer->apb);

      /* And release the buffer container */

      sai_buf_free(priv, bfcontainer);
    }
}

/****************************************************************************
 * Name: sai_schedule
 *
 * Description:
 *   An transfer completion or timeout has occurred.  Schedule processing on
 *   the working thread.
 *
 * Input Parameters:
 *   priv   - SAI state instance
 *   result - The result of the transfer
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - Interrupts are disabled
 *   - The timeout has been canceled.
 *
 ****************************************************************************/

static void sai_schedule(struct stm32_sai_s *priv, int result)
{
  struct sai_buffer_s *bfcontainer;
  int ret;

  /* Move all entries from the act queue to the done queue */

  while (!sq_empty(&priv->act))
    {
      /* Remove the next buffer container from the act list */

      bfcontainer = (struct sai_buffer_s *)sq_remfirst(&priv->act);

      /* Report the result of the transfer */

      bfcontainer->result = result;

      /* Add the completed buffer container to the tail of the done queue */

      sq_addlast((sq_entry_t *)bfcontainer, &priv->done);
    }

  /* If the worker has completed running, then reschedule the working thread.
   * REVISIT:  There may be a race condition here.  So we do nothing is the
   * worker is not available.
   */

  if (work_available(&priv->work))
    {
      /* Schedule the done processing to occur on the worker thread. */

      ret = work_queue(HPWORK, &priv->work, sai_worker, priv, 0);
      if (ret != 0)
        {
          i2serr("ERROR: Failed to queue work: %d\n", ret);
        }
    }
}

/****************************************************************************
 * Name: sai_dma_callback
 *
 * Description:
 *   This callback function is invoked at the completion of the SAI DMA.
 *
 * Input Parameters:
 *   handle - The DMA handler
 *   isr    - The interrupt status of the DMA transfer
 *   arg    - A pointer to the SAI state instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_SAI_DMA
static void sai_dma_callback(DMA_HANDLE handle, uint8_t isr, void *arg)
{
  struct stm32_sai_s *priv = (struct stm32_sai_s *)arg;

  DEBUGASSERT(priv);

  /* Cancel the watchdog timeout */

  wd_cancel(&priv->dog);

  /* Then schedule completion of the transfer to occur on the worker thread */

  sai_schedule(priv, (isr & SAI_DMA_ERROR) ? -EIO : OK);

  /* Start the next pending transfer now to keep the FIFO fed */

  sai_dma_setup(priv);
}
#endif

/****************************************************************************
 * Name: sai_samplerate
 *
 * Description:
 *   Set the I2S RX/TX sample rate.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   rate - The I2S sample rate in samples (not bits) per second
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

static int32_t sai_samplerate(struct i2s_dev_s *dev, uint32_t rate)
{
  struct stm32_sai_s *priv = (struct stm32_sai_s *)dev;
  bool enabled;

  DEBUGASSERT(priv && rate > 0);

  /* The clock configuration may only change with the block stopped */

  if (sai_disable(priv, &enabled) < 0)
    {
      return 0;
    }

  /* Call callback to change the SAI kernel clock */

  if (priv->sampleratecb != NULL)
    {
      priv->frequency = priv->sampleratecb(dev, rate);
    }
  else
    {
      i2sinfo("No Sample Rate CB set!\n");
    }

  /* Save the new sample rate and update the divider */

  priv->samplerate = rate;
  sai_mckdivider(priv);
  sai_enable(priv, enabled);

  return sai_getbitrate(priv);
}

/****************************************************************************
 * Name: sai_datawidth
 *
 * Description:
 *   Set the I2S data width.  The bitrate is determined by
 *   sample_rate * data_width.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   width - The I2S data with in bits.
 *
 * Returned Value:
 *   Returns the resulting bitrate
 *
 ****************************************************************************/

static int32_t sai_datawidth(struct i2s_dev_s *dev, int bits)
{
  struct stm32_sai_s *priv = (struct stm32_sai_s *)dev;
  uint32_t setbits;
  bool enabled;

  DEBUGASSERT(priv && bits >= 8);

  switch (bits)
    {
      case 8:
        setbits = SAI_CR1_DS_8BITS;
        break;

      case 16:
        setbits = SAI_CR1_DS_16BITS;
        break;

      case 32:
        setbits = SAI_CR1_DS_32BITS;
        break;

      default:
        i2serr("ERROR: Unsupported or invalid data width: %d\n", bits);
        return 0;
    }

  if (sai_disable(priv, &enabled) < 0)
    {
      return 0;
    }

  sai_modifyreg(priv, STM32_SAI_CR1_OFFSET, SAI_CR1_DS_MASK, setbits);

  sai_modifyreg(priv, STM32_SAI_FRCR_OFFSET,
                SAI_FRCR_FSALL_MASK | SAI_FRCR_FRL_MASK,
                SAI_FRCR_FSALL(bits) | SAI_FRCR_FRL(bits * 2));

  sai_enable(priv, enabled);

  /* Save the new data width */

  priv->datalen = bits;

  return sai_getbitrate(priv);
}

/****************************************************************************
 * Name: sai_receive
 *
 * Description:
 *   Receive a block of data from I2S.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   apb      - A pointer to the audio buffer in which to receive data
 *   callback - A user provided callback function that will be called at
 *              the completion of the transfer.  The callback will be
 *              performed in the context of the worker thread.
 *   arg      - An opaque argument that will be provided to the callback
 *              when the transfer complete
 *   timeout  - The timeout value to use.  The transfer will be canceled
 *              and an ETIMEDOUT error will be reported if this timeout
 *              elapsed without completion of the DMA transfer.  Units
 *              are system clock ticks.  Zero means no timeout.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.  NOTE:  This function
 *   only enqueues the transfer and returns immediately.  Success here only
 *   means that the transfer was enqueued correctly.
 *
 *   When the transfer is complete, a 'result' value will be provided as
 *   an argument to the callback function that will indicate if the transfer
 *   failed.
 *
 ****************************************************************************/

static int sai_receive(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                       i2s_callback_t callback, void *arg, uint32_t timeout)
{
  struct stm32_sai_s *priv = (struct stm32_sai_s *)dev;
  struct sai_buffer_s *bfcontainer;
  uint32_t mode;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(priv && apb);
  i2sinfo("apb=%p nbytes=%d arg=%p timeout=%" PRId32 "\n",
          apb, apb->nbytes - apb->curbyte, arg, timeout);

  /* Allocate a buffer container in advance */

  bfcontainer = sai_buf_allocate(priv);
  DEBUGASSERT(bfcontainer);

  /* Get exclusive access to the SAI driver data */

  nxmutex_lock(&priv->lock);

  /* Verify not already TX'ing */

  if (priv->txenab)
    {
      i2serr("ERROR: SAI has no receiver\n");
      ret = -EAGAIN;
      goto errout_with_lock;
    }

  /* Get a receive DMA buffer that does not share cache lines */

  ret = sai_rxbuf_prepare(bfcontainer, apb);
  if (ret < 0)
    {
      goto errout_with_lock;
    }

  mode = priv->syncen ? SAI_CR1_MODE_SLAVE_RX : SAI_CR1_MODE_MASTER_RX;
  sai_modifyreg(priv, STM32_SAI_CR1_OFFSET, SAI_CR1_MODE_MASK, mode);
  priv->rxenab = true;

  /* Add a reference to the audio buffer */

  apb_reference(apb);

  /* Initialize the buffer container structure */

  bfcontainer->callback = (void *)callback;
  bfcontainer->timeout  = timeout;
  bfcontainer->arg      = arg;
  bfcontainer->apb      = apb;
  bfcontainer->result   = -EBUSY;

  /* Add the buffer container to the end of the pending queue */

  flags = enter_critical_section();
  sq_addlast((sq_entry_t *)bfcontainer, &priv->pend);

  /* Then start the next transfer.  If there is already a transfer in
   * progress, then this will do nothing.
   */

#ifdef CONFIG_STM32_SAI_DMA
  ret = sai_dma_setup(priv);
#endif
  DEBUGASSERT(ret == OK);
  leave_critical_section(flags);
  nxmutex_unlock(&priv->lock);
  return OK;

errout_with_lock:
  nxmutex_unlock(&priv->lock);
  sai_buf_free(priv, bfcontainer);
  return ret;
}

/****************************************************************************
 * Name: sai_send
 *
 * Description:
 *   Send a block of data on I2S.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   apb      - A pointer to the audio buffer from which to send data
 *   callback - A user provided callback function that will be called at
 *              the completion of the transfer.  The callback will be
 *              performed in the context of the worker thread.
 *   arg      - An opaque argument that will be provided to the callback
 *              when the transfer complete
 *   timeout  - The timeout value to use.  The transfer will be canceled
 *              and an ETIMEDOUT error will be reported if this timeout
 *              elapsed without completion of the DMA transfer.  Units
 *              are system clock ticks.  Zero means no timeout.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.  NOTE:  This function
 *   only enqueues the transfer and returns immediately.  Success here only
 *   means that the transfer was enqueued correctly.
 *
 *   When the transfer is complete, a 'result' value will be provided as
 *   an argument to the callback function that will indicate if the transfer
 *   failed.
 *
 ****************************************************************************/

static int sai_send(struct i2s_dev_s *dev, struct ap_buffer_s *apb,
                    i2s_callback_t callback, void *arg, uint32_t timeout)
{
  struct stm32_sai_s *priv = (struct stm32_sai_s *)dev;
  struct sai_buffer_s *bfcontainer;
  uint32_t mode;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(priv && apb);
  i2sinfo("apb=%p nbytes=%d arg=%p timeout=%" PRId32 "\n",
          apb, apb->nbytes - apb->curbyte, arg, timeout);

  /* Allocate a buffer container in advance */

  bfcontainer = sai_buf_allocate(priv);
  DEBUGASSERT(bfcontainer);

  /* Get exclusive access to the SAI driver data */

  nxmutex_lock(&priv->lock);

  /* Verify not already RX'ing */

  if (priv->rxenab)
    {
      i2serr("ERROR: SAI has no transmitter\n");
      ret = -EAGAIN;
      goto errout_with_lock;
    }

  mode = priv->syncen ? SAI_CR1_MODE_SLAVE_TX : SAI_CR1_MODE_MASTER_TX;
  sai_modifyreg(priv, STM32_SAI_CR1_OFFSET, SAI_CR1_MODE_MASK, mode);
  priv->txenab = true;

  /* The DMA does not snoop the data cache: write back the data to send */

  up_clean_dcache((uintptr_t)&apb->samp[apb->curbyte],
                  (uintptr_t)&apb->samp[apb->nbytes]);

  /* Add a reference to the audio buffer */

  apb_reference(apb);

  /* Initialize the buffer container structure */

  bfcontainer->callback = (void *)callback;
  bfcontainer->timeout  = timeout;
  bfcontainer->arg      = arg;
  bfcontainer->apb      = apb;
  bfcontainer->rxbuf    = NULL;
  bfcontainer->result   = -EBUSY;

  /* Add the buffer container to the end of the pending queue */

  flags = enter_critical_section();
  sq_addlast((sq_entry_t *)bfcontainer, &priv->pend);

  /* Then start the next transfer.  If there is already a transfer in
   * progress, then this will do nothing.
   */

#ifdef CONFIG_STM32_SAI_DMA
  ret = sai_dma_setup(priv);
#endif
  DEBUGASSERT(ret == OK);
  leave_critical_section(flags);
  nxmutex_unlock(&priv->lock);
  return OK;

errout_with_lock:
  nxmutex_unlock(&priv->lock);
  sai_buf_free(priv, bfcontainer);
  return ret;
}

/****************************************************************************
 * Name: sai_buf_allocate
 *
 * Description:
 *   Allocate a buffer container by removing the one at the head of the
 *   free list
 *
 * Input Parameters:
 *   priv - SAI state instance
 *
 * Returned Value:
 *   A non-NULL pointer to the allocate buffer container on success; NULL if
 *   there are no available buffer containers.
 *
 * Assumptions:
 *   The caller does NOT have exclusive access to the SAI state structure.
 *   That would result in a deadlock!
 *
 ****************************************************************************/

static struct sai_buffer_s *sai_buf_allocate(struct stm32_sai_s *priv)
{
  struct sai_buffer_s *bfcontainer;
  irqstate_t flags;

  /* Set aside a buffer container.  By doing this, we guarantee that we will
   * have at least one free buffer container.
   */

  nxsem_wait_uninterruptible(&priv->bufsem);

  /* Get the buffer from the head of the free list */

  flags = enter_critical_section();
  bfcontainer = priv->freelist;
  DEBUGASSERT(bfcontainer);

  /* Unlink the buffer from the freelist */

  priv->freelist = bfcontainer->flink;
  leave_critical_section(flags);
  return bfcontainer;
}

/****************************************************************************
 * Name: sai_buf_free
 *
 * Description:
 *   Free buffer container by adding it to the head of the free list
 *
 * Input Parameters:
 *   priv - SAI state instance
 *   bfcontainer - The buffer container to be freed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The caller has exclusive access to the SAI state structure
 *
 ****************************************************************************/

static void sai_buf_free(struct stm32_sai_s *priv,
                         struct sai_buffer_s *bfcontainer)
{
  irqstate_t flags;

  /* Put the buffer container back on the free list */

  flags = enter_critical_section();
  bfcontainer->flink  = priv->freelist;
  priv->freelist = bfcontainer;
  leave_critical_section(flags);

  /* Wake up any threads waiting for a buffer container */

  nxsem_post(&priv->bufsem);
}

/****************************************************************************
 * Name: sai_buf_initialize
 *
 * Description:
 *   Initialize the buffer container allocator by adding all of the
 *   pre-allocated buffer containers to the free list
 *
 * Input Parameters:
 *   priv - SAI state instance
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called early in SAI initialization so that there are no issues with
 *   concurrency.
 *
 ****************************************************************************/

static void sai_buf_initialize(struct stm32_sai_s *priv)
{
  int i;

  priv->freelist = NULL;
  for (i = 0; i < CONFIG_STM32_SAI_MAXINFLIGHT; i++)
    {
      sai_buf_free(priv, &priv->containers[i]);
    }
}

/****************************************************************************
 * Name: sai_portinitialize
 *
 * Description:
 *   Initialize the selected SAI port in its default state
 *
 * Input Parameters:
 *   priv   - private SAI device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sai_portinitialize(struct stm32_sai_s *priv)
{
  sai_dump_regs(priv, "Before initialization");

  /* Initialize buffering */

  sai_buf_initialize(priv);

  /* Configure the master clock divider */

  sai_mckdivider(priv);

  /* Configure the data width */

  sai_datawidth((struct i2s_dev_s *)priv,
                CONFIG_STM32_SAI_DEFAULT_DATALEN);

#ifdef CONFIG_STM32_SAI_DMA
  /* Get DMA channel */

#ifdef CONFIG_STM32_HAVE_IP_DMA_GPDMA
  priv->dma = stm32_dmachannel(GPDMA_TTYPE_M2P);
#else
  priv->dma = stm32_dmachannel(priv->dma_ch);
#endif
  DEBUGASSERT(priv->dma);

  sai_modifyreg(priv, STM32_SAI_CR1_OFFSET, 0, SAI_CR1_DMAEN);
#endif

  sai_modifyreg(priv, STM32_SAI_CR1_OFFSET, SAI_CR1_SYNCEN_MASK,
                priv->syncen);

  /* The frame below is I2S: drive the data on the falling edge of SCK and
   * sample it on the rising edge (CKSTR = 1), for transmit and receive.
   */

  sai_modifyreg(priv, STM32_SAI_CR1_OFFSET, 0,
                SAI_CR1_OUTDRIV | SAI_CR1_CKSTR);

#ifdef CONFIG_STM32_HAVE_IP_SAI_V2
  /* The master clock is only output when enabled */

  if (priv->syncen == SAI_CR1_SYNCEN_ASYNCH)
    {
      sai_modifyreg(priv, STM32_SAI_CR1_OFFSET, 0, SAI_CR1_MCKEN);
    }
#endif

  sai_modifyreg(priv, STM32_SAI_CR2_OFFSET, SAI_CR2_FTH_MASK,
                SAI_CR2_FTH_1QF);

  sai_modifyreg(priv, STM32_SAI_FRCR_OFFSET,
                SAI_FRCR_FSDEF | SAI_FRCR_FSPOL | SAI_FRCR_FSOFF,
                SAI_FRCR_FSDEF_CHID | SAI_FRCR_FSPOL_LOW |
                  SAI_FRCR_FSOFF_BFB);

  sai_modifyreg(priv, STM32_SAI_SLOTR_OFFSET,
                SAI_SLOTR_NBSLOT_MASK | SAI_SLOTR_SLOTEN_MASK,
                SAI_SLOTR_NBSLOT(2) | SAI_SLOTR_SLOTEN_0 |
                  SAI_SLOTR_SLOTEN_1);

  sai_modifyreg(priv, STM32_SAI_CR1_OFFSET, 0, SAI_CR1_SAIEN);
  sai_dump_regs(priv, "After initialization");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_sai_initialize
 *
 * Description:
 *   Initialize the selected SAI block
 *
 * Input Parameters:
 *   intf - I2S interface number (identifying the "logical" SAI interface)
 *
 * Returned Value:
 *   Valid I2S device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct i2s_dev_s *stm32_sai_initialize(int intf,
                                       stm32_sai_sampleratecb_t sampleratecb)
{
  struct stm32_sai_s *priv;
  irqstate_t flags;

  flags = enter_critical_section();

  switch (intf)
    {
#ifdef CONFIG_STM32_SAI1_A
      case SAI1_BLOCK_A:
        {
          i2sinfo("SAI1 Block A Selected\n");
          priv = &g_sai1a_priv;
          priv->sampleratecb = sampleratecb;

          stm32_configgpio(GPIO_SAI1_SD_A);
#  ifndef CONFIG_STM32_SAI1_A_SYNC_WITH_B
          stm32_configgpio(GPIO_SAI1_FS_A);
          stm32_configgpio(GPIO_SAI1_SCK_A);
          stm32_configgpio(GPIO_SAI1_MCLK_A);
#  endif
          break;
        }
#endif

#ifdef CONFIG_STM32_SAI1_B
      case SAI1_BLOCK_B:
        {
          i2sinfo("SAI1 Block B Selected\n");
          priv = &g_sai1b_priv;
          priv->sampleratecb = sampleratecb;

          stm32_configgpio(GPIO_SAI1_SD_B);
#  ifndef CONFIG_STM32_SAI1_B_SYNC_WITH_A
          stm32_configgpio(GPIO_SAI1_FS_B);
          stm32_configgpio(GPIO_SAI1_SCK_B);
          stm32_configgpio(GPIO_SAI1_MCLK_B);
#  endif
          break;
        }
#endif

#ifdef CONFIG_STM32_SAI2_A
      case SAI2_BLOCK_A:
        {
          i2sinfo("SAI2 Block A Selected\n");
          priv = &g_sai2a_priv;
          priv->sampleratecb = sampleratecb;

          stm32_configgpio(GPIO_SAI2_SD_A);
#  ifndef CONFIG_STM32_SAI2_A_SYNC_WITH_B
          stm32_configgpio(GPIO_SAI2_FS_A);
          stm32_configgpio(GPIO_SAI2_SCK_A);
          stm32_configgpio(GPIO_SAI2_MCLK_A);
#  endif
          break;
        }
#endif

#ifdef CONFIG_STM32_SAI2_B
      case SAI2_BLOCK_B:
        {
          i2sinfo("SAI2 Block B Selected\n");
          priv = &g_sai2b_priv;
          priv->sampleratecb = sampleratecb;

          stm32_configgpio(GPIO_SAI2_SD_B);
#  ifndef CONFIG_STM32_SAI2_B_SYNC_WITH_A
          stm32_configgpio(GPIO_SAI2_FS_B);
          stm32_configgpio(GPIO_SAI2_SCK_B);
          stm32_configgpio(GPIO_SAI2_MCLK_B);
#  endif
          break;
        }
#endif

      default:
        {
          i2sinfo("No SAI interface defined\n");
          goto err;
        }
    }

  sai_portinitialize(priv);
  leave_critical_section(flags);

  return &priv->dev;

err:
  leave_critical_section(flags);
  return NULL;
}

/****************************************************************************
 * Name: stm32_sai_uninitialize
 *
 * Description:
 *   Stop the SAI block and release what stm32_sai_initialize() set up: the
 *   DMA channel and the master clock output.  No transfer may be queued.
 *
 * Input Parameters:
 *   dev - Device structure returned by stm32_sai_initialize()
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_sai_uninitialize(struct i2s_dev_s *dev)
{
  struct stm32_sai_s *priv = (struct stm32_sai_s *)dev;
  uint32_t clrbits = SAI_CR1_DMAEN;
  bool enabled;

  DEBUGASSERT(priv && sq_empty(&priv->pend) && sq_empty(&priv->act));

  wd_cancel(&priv->dog);
  sai_disable(priv, &enabled);

#ifdef CONFIG_STM32_HAVE_IP_SAI_V2
  clrbits |= SAI_CR1_MCKEN;
#endif

  sai_modifyreg(priv, STM32_SAI_CR1_OFFSET, clrbits, 0);

#ifdef CONFIG_STM32_SAI_DMA
  if (priv->dma != NULL)
    {
      stm32_dmastop(priv->dma);
      stm32_dmafree(priv->dma);
      priv->dma = NULL;
    }
#endif

  priv->txenab = false;
  priv->rxenab = false;
}

#endif
