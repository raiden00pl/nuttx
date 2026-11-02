/****************************************************************************
 * boards/arm/common/stm32/src/stm32_gt9xx.c
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

#include <stdio.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/input/gt9xx.h>
#include <nuttx/signal.h>

#include "stm32.h"
#include "stm32_exti.h"
#include "stm32_i2c.h"
#include "stm32_gt9xx.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GT9XX_I2C_FREQUENCY 100000

/* The controller needs about 200 ms after power-on to report its ID */

#define GT9XX_PROBE_RETRIES 50
#define GT9XX_PROBE_DELAY   10000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_gt9xx_s
{
  struct gt9xx_board_s board;   /* Must be first */
  uint32_t intpin;              /* Interrupt GPIO */
  xcpt_t handler;               /* Driver interrupt handler */
  void *arg;                    /* Driver interrupt argument */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int stm32_gt9xx_attach(const struct gt9xx_board_s *state,
                              xcpt_t handler, void *arg);
static void stm32_gt9xx_enable(const struct gt9xx_board_s *state,
                               bool enable);
static int stm32_gt9xx_power(const struct gt9xx_board_s *state, bool on);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct stm32_gt9xx_s g_gt9xx =
{
  .board =
    {
      .irq_attach = stm32_gt9xx_attach,
      .irq_enable = stm32_gt9xx_enable,
      .set_power  = stm32_gt9xx_power,
    },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int stm32_gt9xx_attach(const struct gt9xx_board_s *state,
                              xcpt_t handler, void *arg)
{
  struct stm32_gt9xx_s *priv = (struct stm32_gt9xx_s *)state;

  priv->handler = handler;
  priv->arg     = arg;
  return OK;
}

static void stm32_gt9xx_enable(const struct gt9xx_board_s *state,
                               bool enable)
{
  struct stm32_gt9xx_s *priv = (struct stm32_gt9xx_s *)state;

  stm32_gpiosetevent(priv->intpin, true, false, true,
                     enable ? priv->handler : NULL,
                     enable ? priv->arg : NULL);
}

static int stm32_gt9xx_power(const struct gt9xx_board_s *state, bool on)
{
  /* Power and reset are handled by the board before registration */

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_gt9xx_initialize
 ****************************************************************************/

int board_gt9xx_initialize(int devno, int busno, uint8_t addr,
                           uint32_t intpin)
{
  struct i2c_master_s *i2c;
  struct i2c_config_s config =
  {
    .frequency = GT9XX_I2C_FREQUENCY,
    .address   = addr,
    .addrlen   = 7
  };

  uint8_t reg[2];
  uint8_t id[4];
  char devpath[16];
  int ret;
  int i;

  g_gt9xx.intpin = intpin;
  stm32_configgpio(intpin);

  i2c = stm32_i2cbus_initialize(busno);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  /* Wait until the controller reports its product ID ("9xx") */

  for (i = 0; i < GT9XX_PROBE_RETRIES; i++)
    {
      reg[0] = 0x81;
      reg[1] = 0x40;
      ret = i2c_writeread(i2c, &config, reg, 2, id, sizeof(id));
      if (ret >= 0 && id[0] == '9')
        {
          break;
        }

      nxsig_usleep(GT9XX_PROBE_DELAY);
    }

  if (i == GT9XX_PROBE_RETRIES)
    {
      stm32_i2cbus_uninitialize(i2c);
      return -ENODEV;
    }

  snprintf(devpath, sizeof(devpath), "/dev/input%d", devno);
  ret = gt9xx_register(devpath, i2c, addr, &g_gt9xx.board);
  if (ret < 0)
    {
      stm32_i2cbus_uninitialize(i2c);
    }

  return ret;
}
