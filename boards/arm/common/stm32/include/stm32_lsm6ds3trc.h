/****************************************************************************
 * boards/arm/common/stm32/include/stm32_lsm6ds3trc.h
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

#ifndef __BOARDS_ARM_COMMON_STM32_INCLUDE_STM32_LSM6DS3TRC_H
#define __BOARDS_ARM_COMMON_STM32_INCLUDE_STM32_LSM6DS3TRC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: board_lsm6ds3trc_initialize
 *
 * Description:
 *   Register an LSM6DS3TR-C compatible IMU (LSM6DSL, ISM330DLC) on an I2C
 *   bus as /dev/uorb/sensor_accel<devno> and /dev/uorb/sensor_gyro<devno>.
 *   The driver polls the sensor; its interrupt lines are not used.
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

int board_lsm6ds3trc_initialize(int devno, int busno, uint8_t addr);

#ifdef __cplusplus
}
#endif

#endif /* __BOARDS_ARM_COMMON_STM32_INCLUDE_STM32_LSM6DS3TRC_H */
