/****************************************************************************
 * drivers/sensors/hts221_base.h
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

#ifndef __DRIVERS_SENSORS_HTS221_BASE_H
#define __DRIVERS_SENSORS_HTS221_BASE_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_HTS221_DEBUG
#  define hts221_dbg(x, ...)    _info(x, ##__VA_ARGS__)
#else
#  define hts221_dbg(x, ...)    sninfo(x, ##__VA_ARGS__)
#endif

#define HTS221_WHO_AM_I             0x0f
#define HTS221_AV_CONF              0x10
#define HTS221_CTRL_REG1            0x20
#define HTS221_CTRL_REG2            0x21
#define HTS221_CTRL_REG3            0x22
#define HTS221_STATUS_REG           0x27
#define HTS221_HUM_OUT_L            0x28
#define HTS221_HUM_OUT_H            0x29
#define HTS221_TEMP_OUT_L           0x2a
#define HTS221_TEMP_OUT_H           0x2b

/* Calibration registers */

#define HTS221_CALIB_H0_RH_X2       0x30
#define HTS221_CALIB_H1_RH_X2       0x31
#define HTS221_CALIB_T0_DEGC_X8     0x32
#define HTS221_CALIB_T1_DEGC_X8     0x33
#define HTS221_CALIB_T1_T0_MSB      0x35
#define HTS221_CALIB_H0T0_OUT_L     0x36
#define HTS221_CALIB_H0T0_OUT_H     0x37
#define HTS221_CALIB_H1T0_OUT_L     0x3a
#define HTS221_CALIB_H1T0_OUT_H     0x3b
#define HTS221_CALIB_T0_OUT_L       0x3c
#define HTS221_CALIB_T0_OUT_H       0x3d
#define HTS221_CALIB_T1_OUT_L       0x3e
#define HTS221_CALIB_T1_OUT_H       0x3f

/* HTS221_CTRL_REG1 */

#define HTS221_CTRL_REG1_PD         (1 << 7)
#define HTS221_CTRL_REG1_BDU        (1 << 2)

/* HTS221_CTRL_REG2 */

#define HTS221_CTRL_REG2_BOOT       (1 << 7)
#define HTS221_CTRL_REG2_ONE_SHOT   (1 << 0)

/* HTS221_CTRL_REG3 */

#define HTS221_CTRL_REG3_DRDY_L_H   (1 << 7)
#define HTS221_CTRL_REG3_PP_OD      (1 << 6)
#define HTS221_CTRL_REG3_DRDY_EN    (1 << 2)

/* HTS221_STATUS_REG */

#define HTS221_STATUS_REG_H_DA      (1 << 1)
#define HTS221_STATUS_REG_T_DA      (1 << 0)

#define HTS221_I2C_RETRIES          10

#endif /* __DRIVERS_SENSORS_HTS221_BASE_H */
