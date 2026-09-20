/****************************************************************************
 * arch/arm/src/nrf54l/nrf54l_adc.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/debug.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <arch/barriers.h>

#include "arm_internal.h"
#include "nrf54l_adc.h"

#include "hardware/nrf54l_saadc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nrf54l_adc_s
{
  /* Upper-half callback */

  const struct adc_callback_s *cb;

  /* Channels configuration */

  struct nrf54l_adc_channel_s channels[CONFIG_NRF54L_SAADC_CHANNELS];

  /* Samples buffer */

  int16_t                    buffer[CONFIG_NRF54L_SAADC_CHANNELS]
                             aligned_data(4);
  int16_t                    calibration[2] aligned_data(4);

  uint8_t                    chan_len;   /* Configured channels */
  uint32_t                   base;       /* Base address of ADC register */
  uint32_t                   irq;        /* ADC interrupt */
  bool                       opened;     /* Device is configured */
  bool                       busy;       /* Conversion in progress */
  bool                       rxenabled;  /* Deliver samples */
  uint16_t                   remaining;  /* Software oversamples left */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* ADC Register access */

static inline void nrf54l_adc_putreg(struct nrf54l_adc_s *priv,
                                    uint32_t offset,
                                    uint32_t value);
static inline uint32_t nrf54l_adc_getreg(struct nrf54l_adc_s *priv,
                                        uint32_t offset);

/* ADC helpers */

static int nrf54l_adc_configure(struct nrf54l_adc_s *priv);
static int nrf54l_adc_calibrate(struct nrf54l_adc_s *priv);
static void nrf54l_adc_stop(struct nrf54l_adc_s *priv);
static uint32_t nrf54l_adc_ch_config(const struct nrf54l_adc_channel_s *cfg);
static uint32_t nrf54l_adc_chanpsel(int psel);
static int nrf54l_adc_chancfg(struct nrf54l_adc_s *priv, uint8_t chan,
                             struct nrf54l_adc_channel_s *cfg);
static int nrf54l_adc_isr(int irq, void *context, void *arg);

/* ADC Driver Methods */

static int  nrf54l_adc_bind(struct adc_dev_s *dev,
                           const struct adc_callback_s *callback);
static void nrf54l_adc_reset(struct adc_dev_s *dev);
static int  nrf54l_adc_setup(struct adc_dev_s *dev);
static void nrf54l_adc_shutdown(struct adc_dev_s *dev);
static void nrf54l_adc_rxint(struct adc_dev_s *dev, bool enable);
static int  nrf54l_adc_ioctl(struct adc_dev_s *dev, int cmd,
                            unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ADC interface operations */

static const struct adc_ops_s g_nrf54l_adcops =
{
  .ao_bind        = nrf54l_adc_bind,
  .ao_reset       = nrf54l_adc_reset,
  .ao_setup       = nrf54l_adc_setup,
  .ao_shutdown    = nrf54l_adc_shutdown,
  .ao_rxint       = nrf54l_adc_rxint,
  .ao_ioctl       = nrf54l_adc_ioctl,
};

/* SAADC device */

static struct nrf54l_adc_s g_nrf54l_adcpriv =
{
  .cb         = NULL,
  .base       = NRF54L_SAADC_BASE,
  .irq        = NRF54L_IRQ_SAADC,
};

/* Upper-half ADC device */

static struct adc_dev_s g_nrf54l_adc =
{
  .ad_ops      = &g_nrf54l_adcops,
  .ad_priv     = &g_nrf54l_adcpriv,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_adc_putreg
 *
 * Description:
 *   Put a 32-bit register value by offset
 *
 ****************************************************************************/

static inline void nrf54l_adc_putreg(struct nrf54l_adc_s *priv,
                                    uint32_t offset,
                                    uint32_t value)
{
  DEBUGASSERT(priv);

  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: nrf54l_adc_getreg
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ****************************************************************************/

static inline uint32_t nrf54l_adc_getreg(struct nrf54l_adc_s *priv,
                                        uint32_t offset)
{
  DEBUGASSERT(priv);

  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: nrf54l_adc_isr
 *
 * Description:
 *   Common ADC interrupt service routine
 *
 ****************************************************************************/

static int nrf54l_adc_isr(int irq, void *context, void *arg)
{
  struct adc_dev_s    *dev  = (struct adc_dev_s *) arg;
  struct nrf54l_adc_s *priv = NULL;
  int                 i    = 0;

  DEBUGASSERT(dev);

  priv = (struct nrf54l_adc_s *) dev->ad_priv;
  DEBUGASSERT(priv);

  ainfo("nrf54l_adc_isr\n");

  /* END event */

  if (nrf54l_adc_getreg(priv, NRF54L_SAADC_EVENTS_END_OFFSET) == 1)
    {
      nrf54l_adc_putreg(priv, NRF54L_SAADC_EVENTS_END_OFFSET, 0);
      nrf54l_adc_getreg(priv, NRF54L_SAADC_EVENTS_END_OFFSET);

      /* Stop the local timer and release DMA before reporting samples. */

      nrf54l_adc_stop(priv);
      nrf54l_adc_putreg(priv, NRF54L_SAADC_EVENTS_END_OFFSET, 0);
      nrf54l_adc_putreg(priv, NRF54L_SAADC_EVENTS_DONE_OFFSET, 0);
      nrf54l_adc_putreg(priv, NRF54L_SAADC_INTENCLR_OFFSET, SAADC_INT_DONE);
      nrf54l_adc_getreg(priv, NRF54L_SAADC_EVENTS_END_OFFSET);
      UP_DMB();

      /* Give the ADC data to the ADC driver */

      if (priv->busy && priv->rxenabled && priv->cb != NULL &&
          nrf54l_adc_getreg(priv, NRF54L_SAADC_AMOUNT_OFFSET) ==
          priv->chan_len * sizeof(int16_t))
        {
          for (i = 0; i < priv->chan_len; i += 1)
            {
              priv->cb->au_receive(dev, i, priv->buffer[i]);
            }
        }

      priv->busy = false;
    }
#if defined(CONFIG_NRF54L_SAADC_TASK) && CONFIG_NRF54L_SAADC_OVERSAMPLE > 0
  else if (priv->busy && !priv->channels[0].burst &&
           nrf54l_adc_getreg(priv, NRF54L_SAADC_EVENTS_DONE_OFFSET) != 0)
    {
      /* Without burst, each oversample requires a separate SAMPLE task. */

      nrf54l_adc_putreg(priv, NRF54L_SAADC_EVENTS_DONE_OFFSET, 0);
      nrf54l_adc_getreg(priv, NRF54L_SAADC_EVENTS_DONE_OFFSET);
      if (priv->remaining > 0)
        {
          priv->remaining--;
          nrf54l_adc_putreg(priv, NRF54L_SAADC_TASKS_SAMPLE_OFFSET, 1);
        }
      else
        {
          nrf54l_adc_putreg(priv, NRF54L_SAADC_INTENCLR_OFFSET,
                          SAADC_INT_DONE);
        }
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: nrf54l_adc_configure
 *
 * Description:
 *   Configure ADC
 *
 ****************************************************************************/

static int nrf54l_adc_configure(struct nrf54l_adc_s *priv)
{
  uint32_t regval = 0;

  DEBUGASSERT(priv);

  /* Configure ADC resolution */

  regval = CONFIG_NRF54L_SAADC_RESOLUTION;
  nrf54l_adc_putreg(priv, NRF54L_SAADC_RESOLUTION_OFFSET, regval);

  /* Configure oversampling */

  regval = CONFIG_NRF54L_SAADC_OVERSAMPLE;
  nrf54l_adc_putreg(priv, NRF54L_SAADC_OVERSAMPLE_OFFSET, regval);

#ifndef CONFIG_ARCH_CHIP_NRF54L15
  /* LM20 has global burst control instead of a per-channel CONFIG bit. */

  regval = CONFIG_NRF54L_SAADC_OVERSAMPLE > 0 && priv->channels[0].burst ?
           SAADC_BURST_EN : SAADC_BURST_DIS;
  nrf54l_adc_putreg(priv, NRF54L_SAADC_BURST_OFFSET, regval);
#endif

  /* Configure sample rate */

#if defined(CONFIG_NRF54L_SAADC_TIMER)
  /* Trigger from local timer */

  regval = SAADC_SAMPLERATE_MODE_TIMERS;
  regval |= ((CONFIG_NRF54L_SAADC_TIMER_CC & SAADC_SAMPLERATE_CC_MASK)
             << SAADC_SAMPLERATE_CC_SHIFT);
#elif defined(CONFIG_NRF54L_SAADC_TASK)
  /* Trigger on SAMPLE task */

  regval = SAADC_SAMPLERATE_MODE_TASK;
#else
#  error SAADC trigger not selected
#endif

  nrf54l_adc_putreg(priv, NRF54L_SAADC_SAMPLERATE_OFFSET, regval);
  nrf54l_adc_putreg(priv, NRF54L_SAADC_SHORTS_OFFSET, 0);
  nrf54l_adc_putreg(priv, NRF54L_SAADC_NOISESHAPE_OFFSET,
                  SAADC_NOISESHAPE_DISABLE);

  /* Configure ADC buffer */

  regval = (uintptr_t)priv->buffer;
  nrf54l_adc_putreg(priv, NRF54L_SAADC_PTR_OFFSET, regval);

  regval = priv->chan_len * sizeof(int16_t);
  nrf54l_adc_putreg(priv, NRF54L_SAADC_MAXCNT_OFFSET, regval);

  return OK;
}

/****************************************************************************
 * Name: nrf54l_adc_calibrate
 *
 * Description:
 *   Calibrate ADC
 *
 ****************************************************************************/

static int nrf54l_adc_calibrate(struct nrf54l_adc_s *priv)
{
  /* Clear Event */

  nrf54l_adc_putreg(priv, NRF54L_SAADC_EVENTS_CALDONE_OFFSET, 0);

  /* Start calibration */

  nrf54l_adc_putreg(priv, NRF54L_SAADC_TASKS_CALOFFSET_OFFSET, 1);

  /* Wait for calibration done */

  while (nrf54l_adc_getreg(priv, NRF54L_SAADC_EVENTS_CALDONE_OFFSET) != 1)
    {
    }

  nrf54l_adc_putreg(priv, NRF54L_SAADC_EVENTS_CALDONE_OFFSET, 0);

  /* Latch a scratch buffer and stop to drain post-calibration results,
   * following the Nordic SAADC calibration sequence.
   */

  nrf54l_adc_putreg(priv, NRF54L_SAADC_PTR_OFFSET,
                  (uintptr_t)priv->calibration);
  nrf54l_adc_putreg(priv, NRF54L_SAADC_MAXCNT_OFFSET,
                  sizeof(priv->calibration));
  nrf54l_adc_putreg(priv, NRF54L_SAADC_EVENTS_STARTED_OFFSET, 0);
  UP_DMB();
  nrf54l_adc_putreg(priv, NRF54L_SAADC_TASKS_START_OFFSET, 1);
  while (nrf54l_adc_getreg(priv, NRF54L_SAADC_EVENTS_STARTED_OFFSET) == 0)
    {
    }

  nrf54l_adc_stop(priv);
  nrf54l_adc_putreg(priv, NRF54L_SAADC_EVENTS_END_OFFSET, 0);
  nrf54l_adc_putreg(priv, NRF54L_SAADC_EVENTS_DONE_OFFSET, 0);
  nrf54l_adc_putreg(priv, NRF54L_SAADC_PTR_OFFSET,
                  (uintptr_t)priv->buffer);
  nrf54l_adc_putreg(priv, NRF54L_SAADC_MAXCNT_OFFSET,
                  priv->chan_len * sizeof(int16_t));

  return OK;
}

/****************************************************************************
 * Name: nrf54l_adc_stop
 *
 * Description:
 *   Stop sampling and wait until DMA has released the result buffer.
 *
 ****************************************************************************/

static void nrf54l_adc_stop(struct nrf54l_adc_s *priv)
{
  nrf54l_adc_putreg(priv, NRF54L_SAADC_EVENTS_STOPPED_OFFSET, 0);
  nrf54l_adc_getreg(priv, NRF54L_SAADC_EVENTS_STOPPED_OFFSET);
  nrf54l_adc_putreg(priv, NRF54L_SAADC_TASKS_STOP_OFFSET, 1);
  while (nrf54l_adc_getreg(priv, NRF54L_SAADC_EVENTS_STOPPED_OFFSET) == 0)
    {
    }

  nrf54l_adc_putreg(priv, NRF54L_SAADC_EVENTS_STOPPED_OFFSET, 0);
  nrf54l_adc_getreg(priv, NRF54L_SAADC_EVENTS_STOPPED_OFFSET);
  UP_DMB();
}

/****************************************************************************
 * Name: nrf54l_adc_ch_config
 *
 * Description:
 *   Encode a validated channel configuration.
 *
 ****************************************************************************/

static uint32_t nrf54l_adc_ch_config(const struct nrf54l_adc_channel_s *cfg)
{
  uint32_t regval = 0;

  /* Gain control */

  switch (cfg->gain)
    {
      case NRF54L_ADC_GAIN_2_3:
        {
          regval |= SAADC_CONFIG_GAIN_2P3;
          break;
        }

      case NRF54L_ADC_GAIN_2_5:
        {
          regval |= SAADC_CONFIG_GAIN_2P5;
          break;
        }

      case NRF54L_ADC_GAIN_1_4:
        {
          regval |= SAADC_CONFIG_GAIN_1P4;
          break;
        }

      case NRF54L_ADC_GAIN_1_3:
        {
          regval |= SAADC_CONFIG_GAIN_1P3;
          break;
        }

      case NRF54L_ADC_GAIN_1_2:
        {
          regval |= SAADC_CONFIG_GAIN_1P2;
          break;
        }

      case NRF54L_ADC_GAIN_1:
        {
          regval |= SAADC_CONFIG_GAIN_1;
          break;
        }

      case NRF54L_ADC_GAIN_2:
        {
          regval |= SAADC_CONFIG_GAIN_2;
          break;
        }

      case NRF54L_ADC_GAIN_2_7:
        {
          regval |= SAADC_CONFIG_GAIN_2P7;
          break;
        }

      default:
        {
          aerr("ERROR: invalid cfg->gain: %d\n", cfg->gain);
        }
    }

  /* Reference control */

  switch (cfg->refsel)
    {
      case NRF54L_ADC_REFSEL_INTERNAL:
        {
          regval |= SAADC_CONFIG_REFSEL_INTERNAL;
          break;
        }

      case NRF54L_ADC_REFSEL_EXTERNAL:
        {
          regval |= SAADC_CONFIG_REFSEL_EXTERNAL;
          break;
        }

      default:
        {
          aerr("ERROR: invalid cfg->refsel: %d\n", cfg->refsel);
        }
    }

  /* Acquisition time */

  switch (cfg->tacq)
    {
      case NRF54L_ADC_TACQ_3US:
        {
          regval |= SAADC_CONFIG_TACQ_3US;
          break;
        }

      case NRF54L_ADC_TACQ_5US:
        {
          regval |= SAADC_CONFIG_TACQ_5US;
          break;
        }

      case NRF54L_ADC_TACQ_10US:
        {
          regval |= SAADC_CONFIG_TACQ_10US;
          break;
        }

      case NRF54L_ADC_TACQ_15US:
        {
          regval |= SAADC_CONFIG_TACQ_15US;
          break;
        }

      case NRF54L_ADC_TACQ_20US:
        {
          regval |= SAADC_CONFIG_TACQ_20US;
          break;
        }

      case NRF54L_ADC_TACQ_40US:
        {
          regval |= SAADC_CONFIG_TACQ_40US;
          break;
        }

      default:
        {
          aerr("ERROR: invalid cfg->tacq: %d\n", cfg->tacq);
        }
    }

  /* Use a 2 us conversion time, as on nRF53. */

  regval |= SAADC_CONFIG_TCONV_2US;

  /* Single-ended or differential mode */

  switch (cfg->mode)
    {
      case NRF54L_ADC_MODE_SE:
        {
          regval |= SAADC_CONFIG_MODE_SE;
          break;
        }

      case NRF54L_ADC_MODE_DIFF:
        {
          regval |= SAADC_CONFIG_MODE_DIFF;
          break;
        }

      default:
        {
          aerr("ERROR: invalid cfg->mode: %d\n", cfg->mode);
        }
    }

  /* Burst mode is configured per channel on L15. */

#ifdef CONFIG_ARCH_CHIP_NRF54L15
  switch (cfg->burst)
    {
      case NRF54L_ADC_BURST_DISABLE:
        {
          regval |= SAADC_CONFIG_BURS_DIS;
          break;
        }

      case NRF54L_ADC_BURST_ENABLE:
        {
          if (CONFIG_NRF54L_SAADC_OVERSAMPLE > 0)
            {
              regval |= SAADC_CONFIG_BURS_EN;
            }

          break;
        }

      default:
        {
          aerr("ERROR: invalid cfg->burst: %d\n", cfg->burst);
        }
    }
#endif

  return regval;
}

/****************************************************************************
 * Name: nrf54l_adc_chanpsel
 *
 * Description:
 *   Convert an analog input number to the hardware pin selection.
 *
 ****************************************************************************/

static uint32_t nrf54l_adc_chanpsel(int psel)
{
  uint32_t regval = 0;

  /* AIN0 through AIN7 use different pads on L15 and LM20. */

#ifdef CONFIG_ARCH_CHIP_NRF54L15
  static const uint8_t pins[8] =
  {
    4, 5, 6, 7, 11, 12, 13, 14
  };
#else
  static const uint8_t pins[8] =
  {
    0, 31, 30, 29, 6, 5, 4, 3
  };
#endif

  if (psel >= NRF54L_ADC_IN_IN0 && psel <= NRF54L_ADC_IN_IN7)
    {
      return SAADC_CHPSEL_ANALOG | (1 << SAADC_CHPSEL_PORT_SHIFT) |
             pins[psel - NRF54L_ADC_IN_IN0];
    }

  switch (psel)
    {
      case NRF54L_ADC_IN_NC:
        {
          regval = SAADC_CHPSEL_NC;
          break;
        }

      case NRF54L_ADC_IN_VDD:
        {
          regval = SAADC_CHPSEL_INTERNAL | SAADC_CHPSEL_VDD;
          break;
        }

      case NRF54L_ADC_IN_AVDD:
        {
          regval = SAADC_CHPSEL_INTERNAL | SAADC_CHPSEL_AVDD;
          break;
        }

      case NRF54L_ADC_IN_DVDD:
        {
          regval = SAADC_CHPSEL_INTERNAL | SAADC_CHPSEL_DVDD;
          break;
        }

      default:
        {
          aerr("ERROR: invalid psel: %d\n", psel);
        }
    }

  return regval;
}

/****************************************************************************
 * Name: nrf54l_adc_chancfg
 *
 * Description:
 *   Configure ADC channel
 *
 ****************************************************************************/

static int nrf54l_adc_chancfg(struct nrf54l_adc_s *priv, uint8_t chan,
                             struct nrf54l_adc_channel_s *cfg)
{
  uint32_t regval = 0;
  int      ret    = OK;

  DEBUGASSERT(priv);

  /* Configure positive input */

  regval = nrf54l_adc_chanpsel(cfg->p_psel);
  nrf54l_adc_putreg(priv, NRF54L_SAADC_CHPSELP_OFFSET(chan), regval);

  /* Configure negative input */

  regval = nrf54l_adc_chanpsel(cfg->n_psel);
  nrf54l_adc_putreg(priv, NRF54L_SAADC_CHPSELN_OFFSET(chan), regval);

  /* Get channel configuration */

  regval = nrf54l_adc_ch_config(cfg);

  /* Write channel configuration */

  nrf54l_adc_putreg(priv, NRF54L_SAADC_CHCONFIG_OFFSET(chan), regval);

#ifdef CONFIG_NRF54L_SAADC_LIMITS
  /* Configure limits */

  regval = ((uint32_t)(uint16_t)cfg->limith << SAADC_CHLIMIT_HIGH_SHIFT) |
           ((uint32_t)(uint16_t)cfg->limitl << SAADC_CHLIMIT_LOW_SHIFT);
  nrf54l_adc_putreg(priv, NRF54L_SAADC_CHLIMIT_OFFSET(chan), regval);
#endif

  return ret;
}

/****************************************************************************
 * Name: nrf54l_adc_bind
 *
 * Description:
 *   Bind the upper-half driver callbacks to the lower-half implementation.
 *   This must be called early in order to receive ADC event notifications.
 *
 ****************************************************************************/

static int nrf54l_adc_bind(struct adc_dev_s *dev,
                          const struct adc_callback_s *callback)
{
  struct nrf54l_adc_s *priv = (struct nrf54l_adc_s *) dev->ad_priv;

  DEBUGASSERT(dev);
  DEBUGASSERT(priv);

  if (callback == NULL || callback->au_receive == NULL)
    {
      return -EINVAL;
    }

  priv->cb = callback;

  return OK;
}

/****************************************************************************
 * Name: nrf54l_adc_reset
 *
 * Description:
 *   Reset the ADC device.  Called early to initialize the hardware.
 *   This is called, before adc_setup() and on error conditions.
 *
 ****************************************************************************/

static void nrf54l_adc_reset(struct adc_dev_s *dev)
{
  struct nrf54l_adc_s *priv = (struct nrf54l_adc_s *) dev->ad_priv;

  DEBUGASSERT(dev);
  DEBUGASSERT(priv);

  if (priv->opened)
    {
      nrf54l_adc_shutdown(dev);
    }
}

/****************************************************************************
 * Name: nrf54l_adc_setup
 *
 * Description:
 *   Configure the ADC. This method is called the first time that the ADC
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching ADC interrupts.
 *   Completion interrupts maintain DMA ownership. Sample delivery is
 *   enabled separately by the upper half.
 *
 ****************************************************************************/

static int nrf54l_adc_setup(struct adc_dev_s *dev)
{
  struct nrf54l_adc_s *priv = (struct nrf54l_adc_s *) dev->ad_priv;
  int                 i    = 0;
  int                 ret  = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(priv);

  if (priv->opened || priv->chan_len == 0)
    {
      return -EINVAL;
    }

  /* Disable ADC */

  nrf54l_adc_putreg(priv, NRF54L_SAADC_INTENCLR_OFFSET, SAADC_INT_ALL);
  nrf54l_adc_putreg(priv, NRF54L_SAADC_ENABLE_OFFSET, 0);
  nrf54l_adc_putreg(priv, NRF54L_SAADC_ENABLE_OFFSET, 0);

  for (i = 0; i < 8; i++)
    {
      nrf54l_adc_putreg(priv, NRF54L_SAADC_CHPSELP_OFFSET(i),
                      SAADC_CHPSEL_NC);
      nrf54l_adc_putreg(priv, NRF54L_SAADC_CHPSELN_OFFSET(i),
                      SAADC_CHPSEL_NC);
      nrf54l_adc_putreg(priv, NRF54L_SAADC_EVENTS_CHLIMH_OFFSET(i), 0);
      nrf54l_adc_putreg(priv, NRF54L_SAADC_EVENTS_CHLIML_OFFSET(i), 0);
    }

  /* Configure ADC */

  ret = nrf54l_adc_configure(priv);
  if (ret < 0)
    {
      aerr("ERROR: nrf54l_adc_configure failed: %d\n", ret);
      goto errout;
    }

  /* Configure ADC channels */

  for (i = 0; i < priv->chan_len; i += 1)
    {
      ret = nrf54l_adc_chancfg(priv, i, &priv->channels[i]);
      if (ret < 0)
        {
          aerr("ERROR: chancfg failed: %d %d\n", i, ret);
          goto errout;
        }
    }

  /* Enable ADC */

  nrf54l_adc_putreg(priv, NRF54L_SAADC_ENABLE_OFFSET, 1);

  /* Calibrate ADC */

  ret = nrf54l_adc_calibrate(priv);
  if (ret < 0)
    {
      aerr("ERROR: adc calibration failed: %d\n", ret);
      goto errout;
    }

  /* Attach the ADC interrupt */

  ret = irq_attach(priv->irq, nrf54l_adc_isr, dev);
  if (ret < 0)
    {
      aerr("ERROR: irq_attach failed: %d\n", ret);
      goto errout;
    }

  /* Enable the ADC interrupt */

  priv->opened = true;
  priv->busy = false;
  priv->rxenabled = false;
  nrf54l_adc_putreg(priv, NRF54L_SAADC_INTENSET_OFFSET, SAADC_INT_END);
  up_enable_irq(priv->irq);
  return OK;

errout:
  if (nrf54l_adc_getreg(priv, NRF54L_SAADC_ENABLE_OFFSET) != 0)
    {
      nrf54l_adc_stop(priv);
    }

  nrf54l_adc_putreg(priv, NRF54L_SAADC_ENABLE_OFFSET, 0);
  nrf54l_adc_putreg(priv, NRF54L_SAADC_ENABLE_OFFSET, 0);
  return ret;
}

/****************************************************************************
 * Name: nrf54l_adc_shutdown
 *
 * Description:
 *   Disable the ADC.  This method is called when the ADC device is closed.
 *   This method reverses the operation the setup method.
 *
 ****************************************************************************/

static void nrf54l_adc_shutdown(struct adc_dev_s *dev)
{
  struct nrf54l_adc_s *priv = (struct nrf54l_adc_s *) dev->ad_priv;

  DEBUGASSERT(dev);
  DEBUGASSERT(priv);

  if (!priv->opened)
    {
      return;
    }

  up_disable_irq(priv->irq);
  nrf54l_adc_putreg(priv, NRF54L_SAADC_INTENCLR_OFFSET, SAADC_INT_ALL);
  nrf54l_adc_stop(priv);

  /* Anomaly 101: write ENABLE twice to fully disable the SAADC. */

  nrf54l_adc_putreg(priv, NRF54L_SAADC_ENABLE_OFFSET, 0);
  nrf54l_adc_putreg(priv, NRF54L_SAADC_ENABLE_OFFSET, 0);
  irq_detach(priv->irq);
  priv->busy = false;
  priv->rxenabled = false;
  priv->opened = false;
}

/****************************************************************************
 * Name: nrf54l_adc_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts.
 *
 ****************************************************************************/

static void nrf54l_adc_rxint(struct adc_dev_s *dev, bool enable)
{
  struct nrf54l_adc_s *priv   = (struct nrf54l_adc_s *) dev->ad_priv;

  DEBUGASSERT(dev);
  DEBUGASSERT(priv);

  ainfo("RXINT enable: %d\n", enable ? 1 : 0);

  priv->rxenabled = enable;
}

/****************************************************************************
 * Name: nrf54l_adc_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method.
 *
 ****************************************************************************/

static int nrf54l_adc_ioctl(struct adc_dev_s *dev, int cmd,
                           unsigned long arg)
{
  struct nrf54l_adc_s *priv = (struct nrf54l_adc_s *) dev->ad_priv;
  irqstate_t flags;
  int ret                  = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(priv);

  switch (cmd)
    {
      case ANIOC_TRIGGER:
        {
          flags = enter_critical_section();
          if (!priv->opened || priv->busy)
            {
              ret = priv->opened ? -EBUSY : -EIO;
              leave_critical_section(flags);
              break;
            }

          priv->busy = true;
          nrf54l_adc_putreg(priv, NRF54L_SAADC_EVENTS_STARTED_OFFSET, 0);
          nrf54l_adc_putreg(priv, NRF54L_SAADC_EVENTS_END_OFFSET, 0);
          nrf54l_adc_putreg(priv, NRF54L_SAADC_EVENTS_DONE_OFFSET, 0);
#if defined(CONFIG_NRF54L_SAADC_TASK) && CONFIG_NRF54L_SAADC_OVERSAMPLE > 0
          if (!priv->channels[0].burst)
            {
              priv->remaining = (1 << CONFIG_NRF54L_SAADC_OVERSAMPLE) - 1;
              nrf54l_adc_putreg(priv, NRF54L_SAADC_INTENSET_OFFSET,
                              SAADC_INT_DONE);
            }
#endif

          /* Start ADC */

          UP_DMB();
          nrf54l_adc_putreg(priv, NRF54L_SAADC_TASKS_START_OFFSET, 1);
          while (nrf54l_adc_getreg(priv,
                                 NRF54L_SAADC_EVENTS_STARTED_OFFSET) == 0)
            {
            }

          /* Trigger first sample */

          nrf54l_adc_putreg(priv, NRF54L_SAADC_TASKS_SAMPLE_OFFSET, 1);
          leave_critical_section(flags);
        }
        break;

      case ANIOC_GET_NCHANNELS:
        {
          /* Return the number of configured channels */

          ret = priv->chan_len;
        }
        break;

      default:
        {
          aerr("ERROR: Unknown cmd: %d\n", cmd);
          ret = -ENOTTY;
        }
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_adcinitialize
 *
 * Description:
 *   Initialize the ADC with a validated list of channel configurations.
 *
 * Input Parameters:
 *   chan     - Channel configurations
 *   channels - Number of channels
 *
 * Returned Value:
 *   Valid ADC device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct adc_dev_s *nrf54l_adcinitialize(
    const struct nrf54l_adc_channel_s *chan, int channels)
{
  struct adc_dev_s    *dev  = NULL;
  struct nrf54l_adc_s *priv = NULL;
  int                 i    = 0;
  irqstate_t          flags;
#ifdef CONFIG_NRF54L_SAADC_TIMER
  uint32_t            clocks;
#endif

  if (chan == NULL || channels < 1 ||
      channels > CONFIG_NRF54L_SAADC_CHANNELS)
    {
      return NULL;
    }

#ifdef CONFIG_NRF54L_SAADC_TIMER
  if (channels > 1)
    {
      aerr("ERROR: timer trigger works only for 1 channel!\n");
      return NULL;
    }
#endif

  /* Get device */

  dev = &g_nrf54l_adc;

  /* Get private data */

  priv = (struct nrf54l_adc_s *) dev->ad_priv;

  for (i = 0; i < channels; i++)
    {
      if (chan[i].p_psel == NRF54L_ADC_IN_NC ||
          chan[i].p_psel > NRF54L_ADC_IN_DVDD ||
          chan[i].n_psel > NRF54L_ADC_IN_DVDD ||
          chan[i].tacq > NRF54L_ADC_TACQ_40US ||
          (chan[i].mode == NRF54L_ADC_MODE_DIFF &&
           chan[i].n_psel == NRF54L_ADC_IN_NC) ||
          (chan[i].mode == NRF54L_ADC_MODE_SE &&
           chan[i].n_psel != NRF54L_ADC_IN_NC))
        {
          return NULL;
        }

#if CONFIG_NRF54L_SAADC_OVERSAMPLE > 0
      if (channels > 1 && !chan[i].burst)
        {
          return NULL;
        }
#endif

#ifdef CONFIG_NRF54L_SAADC_TIMER
      /* The timer period must cover acquisition and conversion. */

      clocks = nrf54l_adc_ch_config(&chan[i]);
      clocks = ((clocks & SAADC_CONFIG_TACQ_MASK) >>
                SAADC_CONFIG_TACQ_SHIFT) + 1;
      clocks = 2 * clocks + 32;
      if (chan[i].burst)
        {
          clocks <<= CONFIG_NRF54L_SAADC_OVERSAMPLE;
        }

      if (clocks > CONFIG_NRF54L_SAADC_TIMER_CC)
        {
          return NULL;
        }
#endif

#ifdef CONFIG_NRF54L_SAADC_LIMITS
      if (chan[i].limitl > chan[i].limith)
        {
          return NULL;
        }
#endif
    }

  flags = enter_critical_section();
  if (priv->opened)
    {
      leave_critical_section(flags);
      return NULL;
    }

  /* Copy channels configuration */

  ainfo("channels: %d\n", channels);

  for (i = 0; i < channels; i += 1)
    {
      memcpy(&priv->channels[i], &chan[i],
             sizeof(struct nrf54l_adc_channel_s));
    }

  priv->chan_len = channels;
  leave_critical_section(flags);
  return dev;
}
