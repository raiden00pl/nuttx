/****************************************************************************
 * boards/arm/nrf52/common/src/nrf52_hdc1008.c
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
#include <nuttx/arch.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include "nrf52_i2c.h"
#include <nuttx/sensors/hdc1008.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_hdc1008_initialize
 *
 * Description:
 *   Initialize I2C-based HDC1008.
 *
 ****************************************************************************/

int nrf52_hdc1008_initialize(int devno, int bus, uint8_t addr)
{
  struct hdc1008_config_s  config;
  struct i2c_master_s      *i2c;
  int                       ret = OK;

  sninfo("Initializing LMS6DSL!\n");

  i2c = nrf52_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  sninfo("INFO: Initializing HCD1008 sensor over I2C%d\n", bus);

  config.i2c  = i2c;
  config.addr = addr;

#ifdef CONFIG_SENSORS_HDC1008_UORB
  /* Register sensor as uorb devices */

  ret = hdc1008_register_uorb(devno, &config);
  if (ret < 0)
    {
      snerr("ERROR: Failed to initialize HCD1008 driver\n");
      return -ENODEV;
    }
#else
#  error only HDC1008 UORB supported
#endif

  sninfo("INFO: HCD1008 sensor has been initialized successfully\n");

  return ret;
}
