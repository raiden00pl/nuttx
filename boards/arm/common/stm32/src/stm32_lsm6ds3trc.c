/****************************************************************************
 * boards/arm/common/stm32/src/stm32_lsm6ds3trc.c
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
#include <debug.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/lsm6ds3trc.h>

#include "stm32_i2c.h"
#include "stm32_lsm6ds3trc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_lsm6ds3trc_initialize
 *
 * Description:
 *   Register an LSM6DS3TR-C compatible IMU on an I2C bus.  The driver
 *   polls the sensor from its own threads.
 *
 * Input Parameters:
 *   devno - Device number of the uORB topics
 *   busno - I2C bus the IMU is on
 *   addr  - 7-bit I2C address of the IMU (0x6a or 0x6b)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_lsm6ds3trc_initialize(int devno, int busno, uint8_t addr)
{
  struct lsm6ds3trc_config_s config =
  {
    .int_pin = LSM6DS3TRC_INT1,
    .attach  = NULL,
  };

  struct i2c_master_s *i2c;
  int ret;

  i2c = stm32_i2cbus_initialize(busno);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  ret = lsm6ds3trc_register(i2c, addr, devno, &config);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register LSM6DS3TR-C: %d\n", ret);
    }

  return ret;
}
