/****************************************************************************
 * boards/arm/stm32n6/stm32n6570-dk/src/stm32_adc.c
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
#include <nuttx/debug.h>

#include <nuttx/analog/adc.h>
#include <arch/board/board.h>

#include "chip.h"
#include "stm32_gpio.h"
#include "stm32_adc.h"
#include "stm32n6570-dk.h"

#ifdef CONFIG_STM32_ADC1

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ADC1 channels on the Arduino connector: A1 (PA9, INP10) and A2 (PA10,
 * INP11).
 */

#define ADC1_NCHANNELS 2

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_adc1_chanlist[ADC1_NCHANNELS] =
{
  10, 11
};

static const uint32_t g_adc1_pinlist[ADC1_NCHANNELS] =
{
  GPIO_ADC1_INP10,
  GPIO_ADC1_INP11
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC1 and register it as /dev/adc0.
 *
 ****************************************************************************/

int stm32_adc_setup(void)
{
  static bool initialized = false;
  struct adc_dev_s *adc;
  int ret;
  int i;

  if (initialized)
    {
      return OK;
    }

  /* Configure the pins as analog inputs */

  for (i = 0; i < ADC1_NCHANNELS; i++)
    {
      stm32_configgpio(g_adc1_pinlist[i]);
    }

  adc = stm32_adc_initialize(1, g_adc1_chanlist, ADC1_NCHANNELS);
  if (adc == NULL)
    {
      aerr("ERROR: Failed to get the ADC1 interface\n");
      return -ENODEV;
    }

  ret = adc_register("/dev/adc0", adc);
  if (ret < 0)
    {
      aerr("ERROR: adc_register failed: %d\n", ret);
      return ret;
    }

  initialized = true;
  return OK;
}

#endif /* CONFIG_STM32_ADC1 */
