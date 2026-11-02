/****************************************************************************
 * boards/arm/stm32n6/stm32n6570-dk/src/stm32_wm8904.c
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

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/audio/pcm.h>
#include <nuttx/audio/wm8904.h>
#include <nuttx/i2c/i2c_master.h>

#include <arch/board/board.h>

#include "stm32_i2c.h"
#include "stm32_rcc.h"
#include "stm32_sai.h"
#include "stm32n6570-dk.h"

#ifdef HAVE_WM8904

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IC7 dividers of PLL2 (1536 MHz) for SAI1, which divides them to
 * MCLK = 256 x Fs
 */

#define WM8904_IC7_DIV_48K   STM32_IC7_DIV  /* 12.288 MHz: 48 kHz family */
#define WM8904_IC7_DIV_44K1  136            /* 11.294 MHz: 44.1 kHz family */

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  wm8904_attach(const struct wm8904_lower_s *lower,
                          wm8904_handler_t isr, void *arg);
static bool wm8904_enable(const struct wm8904_lower_s *lower,
                          bool enable);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The SAI is the I2S clock master: it drives MCLK = 256 x Fs, BCLK and
 * LRCLK and the WM8904 runs SYSCLK from MCLK, so mclk is not used.  The
 * codec interrupt is not used.
 */

static const struct wm8904_lower_s g_wm8904_lower =
{
  .frequency = WM8904_I2C_FREQUENCY,
  .address   = WM8904_I2C_ADDRESS,
  .slave     = true,
  .attach    = wm8904_attach,
  .enable    = wm8904_enable,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int wm8904_attach(const struct wm8904_lower_s *lower,
                         wm8904_handler_t isr, void *arg)
{
  return OK;
}

static bool wm8904_enable(const struct wm8904_lower_s *lower, bool enable)
{
  return false;
}

/****************************************************************************
 * Name: wm8904_sampleratecb
 *
 * Description:
 *   Select the SAI1 kernel clock (IC7 from PLL2) for the sample rate
 *   family and return its frequency.
 *
 ****************************************************************************/

static uint32_t wm8904_sampleratecb(struct i2s_dev_s *dev, uint32_t rate)
{
  uint32_t div;

  div = (rate % 11025) == 0 ? WM8904_IC7_DIV_44K1 : WM8904_IC7_DIV_48K;
  stm32_rcc_setic(7, STM32_IC7_SRC, div);

  audinfo("rate %" PRIu32 ": IC7 = PLL2 / %" PRIu32 "\n", rate, div);
  return STM32_PLL2_FREQUENCY / div;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_wm8904_initialize
 *
 * Description:
 *   Initialize SAI1 block A and the WM8904 codec on I2C2 and register the
 *   PCM audio device /dev/audio/pcm<minor>.
 *
 ****************************************************************************/

int stm32_wm8904_initialize(int minor)
{
  struct audio_lowerhalf_s *wm8904;
  struct audio_lowerhalf_s *pcm;
  struct i2c_master_s *i2c;
  struct i2s_dev_s *i2s;
  char devname[12];
  int ret;

  i2c = stm32_i2cbus_initialize(WM8904_I2C_BUS);
  if (i2c == NULL)
    {
      auderr("ERROR: Failed to initialize I2C%d\n", WM8904_I2C_BUS);
      return -ENODEV;
    }

  /* SAI1 block A starts MCLK, which the codec needs to be configured */

  i2s = stm32_sai_initialize(SAI1_BLOCK_A, wm8904_sampleratecb);
  if (i2s == NULL)
    {
      auderr("ERROR: Failed to initialize SAI1 block A\n");
      ret = -ENODEV;
      goto errout_with_i2c;
    }

  wm8904 = wm8904_initialize(i2c, i2s, &g_wm8904_lower);
  if (wm8904 == NULL)
    {
      auderr("ERROR: Failed to initialize the WM8904\n");
      ret = -ENODEV;
      goto errout_with_i2c;
    }

  /* Put a PCM (WAV) decoder in front of the codec */

  pcm = pcm_decode_initialize(wm8904);
  if (pcm == NULL)
    {
      auderr("ERROR: Failed to create the PCM decoder\n");
      ret = -ENODEV;
      goto errout_with_i2c;
    }

  snprintf(devname, sizeof(devname), "pcm%d", minor);
  ret = audio_register(devname, pcm);
  if (ret < 0)
    {
      auderr("ERROR: Failed to register /dev/audio/%s: %d\n", devname, ret);
      goto errout_with_i2c;
    }

  return OK;

errout_with_i2c:
  stm32_i2cbus_uninitialize(i2c);
  return ret;
}

#endif /* HAVE_WM8904 */
