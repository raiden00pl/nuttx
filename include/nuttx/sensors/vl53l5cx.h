/****************************************************************************
 * include/nuttx/sensors/vl53l5cx.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_VL53L5CX_H
#define __INCLUDE_NUTTX_SENSORS_VL53L5CX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#ifdef CONFIG_SENSORS_VL53L5CX

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: vl53l5cx_register
 *
 * Description:
 *   Check that a VL53L5CX answers on the bus and register it as a V4L2
 *   depth capture device (V4L2_PIX_FMT_Z16, 8x8 or 4x4).  The firmware
 *   and the configuration files are loaded at the first stream start.
 *
 * Input Parameters:
 *   devpath - The capture device path, e.g. "/dev/video1"
 *   i2c     - The I2C bus the sensor is on
 *   addr    - The 7-bit I2C address of the sensor (0x29 by default)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int vl53l5cx_register(FAR const char *devpath,
                      FAR struct i2c_master_s *i2c, uint8_t addr);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SENSORS_VL53L5CX */
#endif /* __INCLUDE_NUTTX_SENSORS_VL53L5CX_H */
