/****************************************************************************
 * boards/arm/stm32h7/stm32h735g-dk/src/stm32_touchscreen.c
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

#include <errno.h>
#include <stdio.h>
#include <nuttx/irq.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/input/ft5x06.h>
#include <nuttx/input/gt9xx.h>

#include "stm32_gpio.h"
#include "stm32_i2c.h"
#include "stm32h735g-dk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_STM32_I2C4
#  error "The touchscreen requires CONFIG_STM32_I2C4"
#endif

#define TOUCH_I2C_PORT      4
#define TOUCH_I2C_FREQUENCY 100000
#define GT911_I2C_ADDRESS   0x5d

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_INPUT_GT9XX) || !defined(CONFIG_FT5X06_POLLMODE)
static xcpt_t g_touch_handler;
static void *g_touch_arg;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_INPUT_GT9XX) || !defined(CONFIG_FT5X06_POLLMODE)
static void stm32_touch_enable(bool enable)
{
  stm32_gpiosetevent(GPIO_TOUCH_INT, false, true, true,
                    enable ? g_touch_handler : NULL,
                    enable ? g_touch_arg : NULL);
}
#endif

#ifdef CONFIG_INPUT_GT9XX
static int stm32_gt911_attach(const struct gt9xx_board_s *state,
                             xcpt_t handler, void *arg)
{
  g_touch_handler = handler;
  g_touch_arg = arg;
  return OK;
}

static void stm32_gt911_enable(const struct gt9xx_board_s *state,
                               bool enable)
{
  stm32_touch_enable(enable);
}

static int stm32_gt911_power(const struct gt9xx_board_s *state, bool on)
{
  /* The touch panel supply and reset are not controlled by GPIOs. */

  return OK;
}

static const struct gt9xx_board_s g_gt911_config =
{
  .irq_attach = stm32_gt911_attach,
  .irq_enable = stm32_gt911_enable,
  .set_power  = stm32_gt911_power
};
#endif

#ifdef CONFIG_INPUT_FT5X06
#ifndef CONFIG_FT5X06_POLLMODE
static int stm32_ft5x06_attach(const struct ft5x06_config_s *config,
                              xcpt_t handler, void *arg)
{
  g_touch_handler = handler;
  g_touch_arg = arg;
  return OK;
}

static void stm32_ft5x06_enable(const struct ft5x06_config_s *config,
                               bool enable)
{
  stm32_touch_enable(enable);
}

static void stm32_ft5x06_clear(const struct ft5x06_config_s *config)
{
}
#endif

static void stm32_ft5x06_wakeup(const struct ft5x06_config_s *config)
{
  /* The touch panel wake pin is not connected to the MCU. */
}

static void stm32_ft5x06_nreset(const struct ft5x06_config_s *config,
                               bool state)
{
  /* The touch panel reset is not controlled by a GPIO. */
}

static const struct ft5x06_config_s g_ft5x06_config =
{
  .address   = FT5X06_I2C_ADDRESS,
  .frequency = TOUCH_I2C_FREQUENCY,
#ifndef CONFIG_FT5X06_POLLMODE
  .attach    = stm32_ft5x06_attach,
  .enable    = stm32_ft5x06_enable,
  .clear     = stm32_ft5x06_clear,
#endif
  .wakeup    = stm32_ft5x06_wakeup,
  .nreset    = stm32_ft5x06_nreset,
  .lower     =
    {
      .flags = TOUCH_FLAG_SWAPXY,
    },
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_tsc_setup
 *
 * Description:
 *   Detect and register the GT911 or FT5336 touchscreen on I2C4.  Different
 *   revisions of the Discovery kit ship with different touch controllers.
 *
 ****************************************************************************/

int stm32_tsc_setup(int minor)
{
  struct i2c_master_s *i2c;
  struct i2c_config_s config =
  {
    .frequency = TOUCH_I2C_FREQUENCY,
    .addrlen   = 7
  };

  uint8_t reg[2];
  uint8_t id[4];
#ifdef CONFIG_INPUT_GT9XX
  char devpath[16];
#endif
  int ret = -ENODEV;

  stm32_configgpio(GPIO_TOUCH_INT);
  i2c = stm32_i2cbus_initialize(TOUCH_I2C_PORT);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

#ifdef CONFIG_INPUT_GT9XX
  config.address = GT911_I2C_ADDRESS;
  reg[0] = 0x81;
  reg[1] = 0x40;
  ret = i2c_writeread(i2c, &config, reg, 2, id, sizeof(id));
  if (ret >= 0 && id[0] == '9' && id[1] == '1' && id[2] == '1')
    {
      snprintf(devpath, sizeof(devpath), "/dev/input%d", minor);
      ret = gt9xx_register(devpath, i2c, GT911_I2C_ADDRESS,
                           &g_gt911_config);
      goto out;
    }
#endif

#ifdef CONFIG_INPUT_FT5X06
  config.address = FT5X06_I2C_ADDRESS;
  reg[0] = 0xa8; /* FocalTech vendor ID */
  ret = i2c_writeread(i2c, &config, reg, 1, id, 1);
  if (ret >= 0)
    {
      ret = ft5x06_register(i2c, &g_ft5x06_config, minor);
      goto out;
    }
#endif

  ret = -ENODEV;

out:
  if (ret < 0)
    {
      stm32_i2cbus_uninitialize(i2c);
    }

  return ret;
}
