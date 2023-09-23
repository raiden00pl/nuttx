/****************************************************************************
 * boards/arm/nrf52/common/src/nrf52_bh1745nuc.c
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
#include <debug.h>
#include <stdio.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/bh1745nuc.h>

#include "nrf52_i2c.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_bh1745nuc_init
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as /dev/lightN
 *   busno - The I2C bus number
 *   addr  - The I2C address
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int nrf52_bh1745nuc_init(int devno, int busno, uint8_t addr)
{
  struct i2c_master_s *i2c;
  int ret;

  sninfo("Initializing BH1745NUC!\n");

  /* Initialize I2C */

  i2c = stm32_i2cbus_initialize(busno);
  if (!i2c)
    {
      return -ENODEV;
    }

  /* Then register the barometer sensor */

  ret = bh1745nuc_register_uorb(devno, i2c, addr);
  if (ret < 0)
    {
      snerr("ERROR: Error registering BH1745NUC\n");
    }

  return ret;
}
