/****************************************************************************
 * arch/arm/src/stm32n6/stm32_dcmipp.c
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

/* DCMIPP capture interface for raw Bayer MIPI CSI-2 sensors.
 *
 * Only pipe1 (the main pipe) is used:
 *
 *   CSI-2 VC0 -> black level -> exposure (white balance gains) ->
 *   demosaicing -> colour correction -> decimation -> downsize ->
 *   gamma -> RGB565 pixel packer -> memory
 *
 * The image processing is static: the black level, white balance gains
 * and colour matrix come from the board.  There is no automatic exposure
 * or white balance.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/param.h>
#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/cache.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/signal.h>
#include <nuttx/video/imgdata.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/stm32n6xxx_dcmipp.h"
#include "hardware/stm32n6xxx_rcc.h"
#include "stm32_csi.h"
#include "stm32_dcmipp.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Maximum downsize factor; larger reductions use the decimation first */

#define DCMIPP_DSIZE_MAX         8

/* Demosaicing filter strengths: 0, no sensor specific tuning */

#define DCMIPP_DM_PEAK           0
#define DCMIPP_DM_LINEV          0
#define DCMIPP_DM_LINEH          0
#define DCMIPP_DM_EDGE           0

/* Pipe1 interrupts */

#define DCMIPP_P1_INTS           (DCMIPP_CMINT_ATXERR | DCMIPP_CMINT_P1FRAME | \
                                  DCMIPP_CMINT_P1OVR)

/* Timeout waiting for the end of a capture, in milliseconds */

#define DCMIPP_STOP_TIMEOUT      100

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_dcmipp_s
{
  struct imgdata_s data;              /* Must be first */
  FAR const struct stm32_dcmipp_config_s *config;
  imgdata_capture_t callback;         /* Frame completion callback */
  FAR void *arg;                      /* Callback argument */
  uintptr_t buf;                      /* Buffer receiving the frame */
  uint32_t framesize;                 /* Bytes in one output frame */
  bool capturing;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int dcmipp_init(FAR struct imgdata_s *data);
static int dcmipp_uninit(FAR struct imgdata_s *data);
static int dcmipp_set_buf(FAR struct imgdata_s *data,
                          uint8_t nr_datafmts,
                          FAR imgdata_format_t *datafmts,
                          FAR uint8_t *addr, uint32_t size);
static int dcmipp_validate_frame_setting(FAR struct imgdata_s *data,
                                         uint8_t nr_datafmts,
                                         FAR imgdata_format_t *datafmts,
                                         FAR imgdata_interval_t *interval);
static int dcmipp_start_capture(FAR struct imgdata_s *data,
                                uint8_t nr_datafmts,
                                FAR imgdata_format_t *datafmts,
                                FAR imgdata_interval_t *interval,
                                FAR imgdata_capture_t callback,
                                FAR void *arg);
static int dcmipp_stop_capture(FAR struct imgdata_s *data);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct imgdata_ops_s g_dcmipp_ops =
{
  .init                   = dcmipp_init,
  .uninit                 = dcmipp_uninit,
  .set_buf                = dcmipp_set_buf,
  .validate_frame_setting = dcmipp_validate_frame_setting,
  .start_capture          = dcmipp_start_capture,
  .stop_capture           = dcmipp_stop_capture,
};

static struct stm32_dcmipp_s g_dcmipp =
{
  .data =
    {
      .ops = &g_dcmipp_ops,
    },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dcmipp_gain
 *
 * Description:
 *   Convert a gain (128 = 1.0) to the exposure block shift and multiplier
 *   fields: gain = MULT * 2^SHF / 128 with MULT < 256.
 *
 ****************************************************************************/

static uint32_t dcmipp_gain(uint32_t gain, unsigned int multshift,
                            unsigned int shfshift)
{
  uint32_t shift = 0;

  while (gain >= 256 && shift < 7)
    {
      gain >>= 1;
      shift++;
    }

  return ((gain & 0xff) << multshift) | (shift << shfshift);
}

/****************************************************************************
 * Name: dcmipp_coef
 *
 * Description:
 *   Pack two signed colour conversion coefficients (256 = 1.0).
 *
 ****************************************************************************/

static uint32_t dcmipp_coef(int16_t low, int16_t high)
{
  uint32_t regval;

  regval  = ((uint32_t)low & DCMIPP_P1CC_COEF_MASK) << DCMIPP_P1CC_LOW_SHIFT;
  regval |= ((uint32_t)high & DCMIPP_P1CC_COEF_MASK) <<
            DCMIPP_P1CC_HIGH_SHIFT;
  return regval;
}

/****************************************************************************
 * Name: dcmipp_configisp
 *
 * Description:
 *   Program the static pipe1 image processing.
 *
 ****************************************************************************/

static void dcmipp_configisp(FAR const struct stm32_dcmipp_config_s *cfg)
{
  uint32_t regval;

  /* Black level */

  if (cfg->blacklevel != 0)
    {
      regval = DCMIPP_P1BLCCR_ENABLE |
               ((uint32_t)cfg->blacklevel << DCMIPP_P1BLCCR_BLCR_SHIFT) |
               ((uint32_t)cfg->blacklevel << DCMIPP_P1BLCCR_BLCG_SHIFT) |
               ((uint32_t)cfg->blacklevel << DCMIPP_P1BLCCR_BLCB_SHIFT);
    }
  else
    {
      regval = 0;
    }

  putreg32(regval, STM32_DCMIPP_P1BLCCR);

  /* White balance gains in the exposure block */

  putreg32(dcmipp_gain(cfg->gain[1], DCMIPP_P1EXCR2_MULTG_SHIFT,
                       DCMIPP_P1EXCR2_SHFG_SHIFT) |
           dcmipp_gain(cfg->gain[2], DCMIPP_P1EXCR2_MULTB_SHIFT,
                       DCMIPP_P1EXCR2_SHFB_SHIFT),
           STM32_DCMIPP_P1EXCR2);
  putreg32(dcmipp_gain(cfg->gain[0], DCMIPP_P1EXCR1_MULTR_SHIFT,
                       DCMIPP_P1EXCR1_SHFR_SHIFT) | DCMIPP_P1EXCR1_ENABLE,
           STM32_DCMIPP_P1EXCR1);

  /* Demosaicing */

  putreg32(DCMIPP_P1DMCR_ENABLE |
           ((uint32_t)cfg->bayer << DCMIPP_P1DMCR_TYPE_SHIFT) |
           (DCMIPP_DM_PEAK << DCMIPP_P1DMCR_PEAK_SHIFT) |
           (DCMIPP_DM_LINEV << DCMIPP_P1DMCR_LINEV_SHIFT) |
           (DCMIPP_DM_LINEH << DCMIPP_P1DMCR_LINEH_SHIFT) |
           ((uint32_t)DCMIPP_DM_EDGE << DCMIPP_P1DMCR_EDGE_SHIFT),
           STM32_DCMIPP_P1DMCR);

  /* Colour correction matrix, no offsets and no clamping */

  putreg32(dcmipp_coef(cfg->ccm[0][0], cfg->ccm[0][1]),
           STM32_DCMIPP_P1CCRR1);
  putreg32(dcmipp_coef(cfg->ccm[0][2], 0), STM32_DCMIPP_P1CCRR2);
  putreg32(dcmipp_coef(cfg->ccm[1][0], cfg->ccm[1][1]),
           STM32_DCMIPP_P1CCGR1);
  putreg32(dcmipp_coef(cfg->ccm[1][2], 0), STM32_DCMIPP_P1CCGR2);
  putreg32(dcmipp_coef(cfg->ccm[2][0], cfg->ccm[2][1]),
           STM32_DCMIPP_P1CCBR1);
  putreg32(dcmipp_coef(cfg->ccm[2][2], 0), STM32_DCMIPP_P1CCBR2);
  putreg32(DCMIPP_P1CCCR_ENABLE, STM32_DCMIPP_P1CCCR);

  /* Gamma */

  putreg32(cfg->gamma ? DCMIPP_P1GMCR_ENABLE : 0, STM32_DCMIPP_P1GMCR);
}

/****************************************************************************
 * Name: dcmipp_configscaler
 *
 * Description:
 *   Scale the sensor frame to the output size: decimate by powers of two
 *   until the remaining ratio fits the downsizer.
 *
 ****************************************************************************/

static void dcmipp_configscaler(FAR const struct stm32_dcmipp_config_s *cfg,
                                uint32_t width, uint32_t height)
{
  uint32_t inw = cfg->width;
  uint32_t inh = cfg->height;
  uint32_t hdec = 0;
  uint32_t vdec = 0;
  uint32_t hratio;
  uint32_t vratio;
  uint32_t hdiv;
  uint32_t vdiv;

  /* The whole sensor frame is used, no crop */

  putreg32(0, STM32_DCMIPP_P1CRSZR);

  while (width * DCMIPP_DSIZE_MAX < inw && hdec < 3)
    {
      hdec++;
      inw /= 2;
    }

  while (height * DCMIPP_DSIZE_MAX < inh && vdec < 3)
    {
      vdec++;
      inh /= 2;
    }

  if (hdec != 0 || vdec != 0)
    {
      putreg32(DCMIPP_P1DCCR_ENABLE | DCMIPP_P1DCCR_HDEC(hdec) |
               DCMIPP_P1DCCR_VDEC(vdec), STM32_DCMIPP_P1DCCR);
    }
  else
    {
      putreg32(0, STM32_DCMIPP_P1DCCR);
    }

  if (inw == width && inh == height)
    {
      putreg32(0, STM32_DCMIPP_P1DSCR);
      return;
    }

  /* Ratio: 8192 * src / dst; division factor: 1024 * dst / src */

  hratio = MIN(inw * 8192 / width, 65535);
  vratio = MIN(inh * 8192 / height, 65535);
  hdiv   = MIN(width * 1024 / inw, 1023);
  vdiv   = MIN(height * 1024 / inh, 1023);

  putreg32((hratio << DCMIPP_P1DSRTIOR_HRATIO_SHIFT) |
           (vratio << DCMIPP_P1DSRTIOR_VRATIO_SHIFT),
           STM32_DCMIPP_P1DSRTIOR);
  putreg32((width << DCMIPP_P1DSSZR_HSIZE_SHIFT) |
           (height << DCMIPP_P1DSSZR_VSIZE_SHIFT), STM32_DCMIPP_P1DSSZR);
  putreg32(DCMIPP_P1DSCR_ENABLE | (hdiv << DCMIPP_P1DSCR_HDIV_SHIFT) |
           (vdiv << DCMIPP_P1DSCR_VDIV_SHIFT), STM32_DCMIPP_P1DSCR);
}

/****************************************************************************
 * Name: dcmipp_interrupt
 ****************************************************************************/

static int dcmipp_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct stm32_dcmipp_s *priv = arg;
  struct timespec ts;
  struct timeval tv;
  uint32_t status;

  status = getreg32(STM32_DCMIPP_CMSR2) & getreg32(STM32_DCMIPP_CMIER);
  putreg32(status, STM32_DCMIPP_CMFCR);

  if ((status & DCMIPP_CMINT_ATXERR) != 0)
    {
      verr("ERROR: DCMIPP AXI transfer error\n");
    }

  if ((status & DCMIPP_CMINT_P1OVR) != 0)
    {
      /* Report the first overrun only; it is re-armed by the next start */

      modifyreg32(STM32_DCMIPP_CMIER, DCMIPP_CMINT_P1OVR, 0);
      verr("ERROR: DCMIPP pipe1 overrun\n");
    }

  if ((status & DCMIPP_CMINT_P1FRAME) != 0 && priv->capturing &&
      priv->callback != NULL)
    {
      /* The frame was written by the DCMIPP behind the data cache */

      up_invalidate_dcache(priv->buf, priv->buf + priv->framesize);

      clock_systime_timespec(&ts);
      tv.tv_sec  = ts.tv_sec;
      tv.tv_usec = ts.tv_nsec / 1000;

      /* The callback hands in the next buffer with set_buf() or stops the
       * capture.
       */

      priv->callback(0, priv->framesize, &tv, priv->arg);
    }

  return OK;
}

/****************************************************************************
 * Name: dcmipp_init
 ****************************************************************************/

static int dcmipp_init(FAR struct imgdata_s *data)
{
  FAR struct stm32_dcmipp_s *priv = (FAR struct stm32_dcmipp_s *)data;
  FAR const struct stm32_dcmipp_config_s *cfg = priv->config;
  int ret;

  /* Reset the DCMIPP and the CSI-2 host */

  putreg32(RCC_APB5RSTR_DCMIPPRST | RCC_APB5RSTR_CSIRST,
           STM32_RCC_APB5RSTSR);
  putreg32(RCC_APB5RSTR_DCMIPPRST | RCC_APB5RSTR_CSIRST,
           STM32_RCC_APB5RSTCR);

  /* Serial input, pipe1 takes the raw data type of virtual channel 0 */

  putreg32(0, STM32_DCMIPP_PRCR);
  putreg32(DCMIPP_CMCR_INSEL, STM32_DCMIPP_CMCR);
  putreg32(DCMIPP_P1FSCR_DTIDA(DCMIPP_DT_RAW8 + (cfg->bpp - 8) / 2) |
           DCMIPP_P1FSCR_DTMODE_DTIDA | DCMIPP_P1FSCR_VC(0),
           STM32_DCMIPP_P1FSCR);

  dcmipp_configisp(cfg);

  /* Start the D-PHY and virtual channel 0 of the CSI-2 host */

  ret = stm32_csi_start(cfg->lanes, cfg->bitrate, cfg->bpp);
  if (ret < 0)
    {
      return ret;
    }

  priv->capturing = false;
  priv->callback  = NULL;
  up_enable_irq(STM32_IRQ_DCMIPP);
  return OK;
}

/****************************************************************************
 * Name: dcmipp_uninit
 ****************************************************************************/

static int dcmipp_uninit(FAR struct imgdata_s *data)
{
  dcmipp_stop_capture(data);
  up_disable_irq(STM32_IRQ_DCMIPP);
  stm32_csi_stop();
  return OK;
}

/****************************************************************************
 * Name: dcmipp_set_buf
 ****************************************************************************/

static int dcmipp_set_buf(FAR struct imgdata_s *data,
                          uint8_t nr_datafmts,
                          FAR imgdata_format_t *datafmts,
                          FAR uint8_t *addr, uint32_t size)
{
  FAR struct stm32_dcmipp_s *priv = (FAR struct stm32_dcmipp_s *)data;
  uint32_t framesize;

  framesize = datafmts[IMGDATA_FMT_MAIN].width *
              datafmts[IMGDATA_FMT_MAIN].height * 2;

  /* The buffer must be cache-line aligned so that invalidating it cannot
   * discard unrelated data.
   */

  if (((uintptr_t)addr & (ARMV8M_DCACHE_LINESIZE - 1)) != 0 ||
      size < framesize)
    {
      return -EINVAL;
    }

  priv->buf       = (uintptr_t)addr;
  priv->framesize = framesize;

  /* Drop any cached lines before the DCMIPP writes the buffer.  The
   * address is taken at the start of the next frame.
   */

  up_invalidate_dcache(priv->buf, priv->buf + framesize);
  putreg32(priv->buf, STM32_DCMIPP_P1PPM0AR1);
  return OK;
}

/****************************************************************************
 * Name: dcmipp_validate_frame_setting
 ****************************************************************************/

static int dcmipp_validate_frame_setting(FAR struct imgdata_s *data,
                                         uint8_t nr_datafmts,
                                         FAR imgdata_format_t *datafmts,
                                         FAR imgdata_interval_t *interval)
{
  FAR struct stm32_dcmipp_s *priv = (FAR struct stm32_dcmipp_s *)data;
  FAR const struct stm32_dcmipp_config_s *cfg = priv->config;
  FAR imgdata_format_t *fmt;

  if (nr_datafmts != 1 || datafmts == NULL)
    {
      return -EINVAL;
    }

  fmt = &datafmts[IMGDATA_FMT_MAIN];
  if (fmt->pixelformat != IMGDATA_PIX_FMT_RGB565)
    {
      return -EINVAL;
    }

  /* The line pitch must be a multiple of 16 bytes; at most 1/64 of the
   * sensor size (1/8 decimation and 1/8 downsize).
   */

  if ((fmt->width & 7) != 0 || fmt->width > cfg->width ||
      fmt->height > cfg->height ||
      fmt->width * 64 < cfg->width || fmt->height * 64 < cfg->height)
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: dcmipp_start_capture
 ****************************************************************************/

static int dcmipp_start_capture(FAR struct imgdata_s *data,
                                uint8_t nr_datafmts,
                                FAR imgdata_format_t *datafmts,
                                FAR imgdata_interval_t *interval,
                                FAR imgdata_capture_t callback,
                                FAR void *arg)
{
  FAR struct stm32_dcmipp_s *priv = (FAR struct stm32_dcmipp_s *)data;
  FAR imgdata_format_t *fmt = &datafmts[IMGDATA_FMT_MAIN];
  irqstate_t flags;
  int ret;

  ret = dcmipp_validate_frame_setting(data, nr_datafmts, datafmts,
                                      interval);
  if (ret < 0)
    {
      return ret;
    }

  dcmipp_configscaler(priv->config, fmt->width, fmt->height);

  putreg32(DCMIPP_P1PPCR_FORMAT_RGB565, STM32_DCMIPP_P1PPCR);
  putreg32(fmt->width * 2, STM32_DCMIPP_P1PPM0PR);

  flags = enter_critical_section();

  priv->callback  = callback;
  priv->arg       = arg;
  priv->capturing = true;

  /* Continuous capture of every frame */

  putreg32(0, STM32_DCMIPP_P1FCTCR);
  putreg32(DCMIPP_P1_INTS, STM32_DCMIPP_CMFCR);
  modifyreg32(STM32_DCMIPP_CMIER, 0, DCMIPP_P1_INTS);
  modifyreg32(STM32_DCMIPP_P1FSCR, 0, DCMIPP_P1FSCR_PIPEN);
  modifyreg32(STM32_DCMIPP_P1FCTCR, 0, DCMIPP_P1FCTCR_CPTREQ);

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: dcmipp_stop_capture
 ****************************************************************************/

static int dcmipp_stop_capture(FAR struct imgdata_s *data)
{
  FAR struct stm32_dcmipp_s *priv = (FAR struct stm32_dcmipp_s *)data;
  irqstate_t flags;
  int i;

  flags = enter_critical_section();

  priv->capturing = false;
  modifyreg32(STM32_DCMIPP_P1FCTCR, DCMIPP_P1FCTCR_CPTREQ, 0);
  modifyreg32(STM32_DCMIPP_CMIER, DCMIPP_P1_INTS, 0);
  putreg32(DCMIPP_P1_INTS, STM32_DCMIPP_CMFCR);

  leave_critical_section(flags);

  /* The capture ends with the current frame.  The capture framework may
   * stop from the frame interrupt, where it is already over.
   */

  if (!up_interrupt_context())
    {
      for (i = 0; i < DCMIPP_STOP_TIMEOUT; i++)
        {
          if ((getreg32(STM32_DCMIPP_CMSR1) & DCMIPP_CMSR1_P1CPTACT) == 0)
            {
              break;
            }

          nxsig_usleep(1000);
        }

      modifyreg32(STM32_DCMIPP_P1FSCR, DCMIPP_P1FSCR_PIPEN, 0);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dcmipp_initialize
 ****************************************************************************/

FAR struct imgdata_s *
stm32_dcmipp_initialize(FAR const struct stm32_dcmipp_config_s *config)
{
  FAR struct stm32_dcmipp_s *priv = &g_dcmipp;

  if (config == NULL || config->bpp < 8 || config->bpp > 14 ||
      (config->bpp & 1) != 0 || config->bayer > STM32_DCMIPP_BAYER_BGGR)
    {
      return NULL;
    }

  priv->config = config;

  if (irq_attach(STM32_IRQ_DCMIPP, dcmipp_interrupt, priv) < 0)
    {
      return NULL;
    }

  return &priv->data;
}
