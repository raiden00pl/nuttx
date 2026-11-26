/****************************************************************************
 * boards/arm/common/stm32/src/stm32_eeprma1.c
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
#include <stdbool.h>
#include <stdint.h>
#include <syslog.h>

#include <nuttx/eeprom/eeprom.h>
#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "stm32.h"
#include "stm32_spi.h"
#include "stm32_eeprma1.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(GPIO_EEPRMA1_CS_M95M01) || !defined(GPIO_EEPRMA1_CS_M95256) || \
    !defined(GPIO_EEPRMA1_CS_M95040) || !defined(GPIO_EEPRMA1_W) || \
    !defined(GPIO_EEPRMA1_HOLD)
#  error "board.h must define the X-NUCLEO-EEPRMA1 GPIOs"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct eeprma1_dev_s
{
  uint32_t              cs;      /* Chip select GPIO */
  enum eeprom_25xx_e    type;    /* ee25xx geometry */
  const char           *path;    /* Device path */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The index in this table is the n of SPIDEV_EEPROM(n) */

static const struct eeprma1_dev_s g_eeprma1_devs[] =
{
  {
    GPIO_EEPRMA1_CS_M95M01, EEPROM_AT25M01, "/dev/eeprom0"
  },
  {
    GPIO_EEPRMA1_CS_M95256, EEPROM_25XX256, "/dev/eeprom1"
  },
  {
    GPIO_EEPRMA1_CS_M95040, EEPROM_25XX040, "/dev/eeprom2"
  },
};

#define EEPRMA1_NDEVS (sizeof(g_eeprma1_devs) / sizeof(g_eeprma1_devs[0]))

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_eeprma1_select
 ****************************************************************************/

void board_eeprma1_select(uint32_t devid, bool selected)
{
  unsigned int i;

  for (i = 0; i < EEPRMA1_NDEVS; i++)
    {
      if (devid == SPIDEV_EEPROM(i))
        {
          stm32_gpiowrite(g_eeprma1_devs[i].cs, !selected);
          return;
        }
    }
}

/****************************************************************************
 * Name: board_eeprma1_initialize
 ****************************************************************************/

int board_eeprma1_initialize(int busno)
{
  struct spi_dev_s *spi;
  unsigned int i;
  int ret;

  /* Chip selects inactive, write protect and hold released */

  for (i = 0; i < EEPRMA1_NDEVS; i++)
    {
      stm32_configgpio(g_eeprma1_devs[i].cs);
      stm32_gpiowrite(g_eeprma1_devs[i].cs, true);
    }

  stm32_configgpio(GPIO_EEPRMA1_W);
  stm32_gpiowrite(GPIO_EEPRMA1_W, true);
  stm32_configgpio(GPIO_EEPRMA1_HOLD);
  stm32_gpiowrite(GPIO_EEPRMA1_HOLD, true);

  spi = stm32_spibus_initialize(busno);
  if (spi == NULL)
    {
      return -ENODEV;
    }

  for (i = 0; i < EEPRMA1_NDEVS; i++)
    {
      ret = ee25xx_initialize(spi, SPIDEV_EEPROM(i),
                              (char *)g_eeprma1_devs[i].path,
                              g_eeprma1_devs[i].type, false);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: %s failed: %d\n",
                 g_eeprma1_devs[i].path, ret);
          return ret;
        }
    }

  return OK;
}
