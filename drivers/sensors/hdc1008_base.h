/****************************************************************************
 * drivers/sensors/hdc1008_base.h
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

#ifndef __DRIVERS_SENSORS_HDC1008_BASE_H
#define __DRIVERS_SENSORS_HDC1008_BASE_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/


#ifdef CONFIG_HDC1008_DEBUG
#  define hdc1008_dbg(x, ...)    _info(x, ##__VA_ARGS__)
#else
#  define hdc1008_dbg(x, ...)    sninfo(x, ##__VA_ARGS__)
#endif

/* Macros to convert raw temperature and humidity to real values. Temperature
 * is scaled by 100, humidity by 10.
 */

#define RAW_TO_TEMP(x) (((x) * 16500 / 65536) - 4000)
#define RAW_TO_RH(x)   ((x) * 1000 / 65536)

/* Resolution for measurements. 8-bit is only valid for humidity. */

#define CONFIGURATION_RES_14BIT  0x00
#define CONFIGURATION_RES_11BIT  0x01
#define CONFIGURATION_RES_8BIT   0x02

/* HDC1008 registers */

#define HDC1008_REG_TEMPERATURE     0x00
#define HDC1008_REG_HUMIDITY        0x01
#define HDC1008_REG_CONFIGURATION   0x02
#define HDC1008_REG_SERIALID_0      0xFB /* Bits 0-15: Bit 24-39 of serial */
#define HDC1008_REG_SERIALID_1      0xFC /* Bits 0-15: Bit 8-23 of serial */
#define HDC1008_REG_SERIALID_2      0xFD /* Bits 7-15: Bit 0-7 of serial */

/* Configuration register bits */

#define HDC1008_CONFIGURATION_HRES_SHIFT      (8)       /* Bits 8-9: Humidity resolution */
#define HDC1008_CONFIGURATION_HRES_MASK       (0x03 << HDC1008_CONFIGURATION_HRES_SHIFT)
#  define HDC1008_CONFIGURATION_HRES_14BIT    (CONFIGURATION_RES_14BIT << HDC1008_CONFIGURATION_HRES_SHIFT)
#  define HDC1008_CONFIGURATION_HRES_11BIT    (CONFIGURATION_RES_11BIT << HDC1008_CONFIGURATION_HRES_SHIFT)
#  define HDC1008_CONFIGURATION_HRES_8BIT     (CONFIGURATION_RES_8BIT  << HDC1008_CONFIGURATION_HRES_SHIFT)
#define HDC1008_CONFIGURATION_TRES_SHIFT      (10)      /* Bit 10: Temperature resolution */
#define HDC1008_CONFIGURATION_TRES_MASK       (0x01 << HDC1008_CONFIGURATION_TRES_SHIFT)
#  define HDC1008_CONFIGURATION_TRES_14BIT    (CONFIGURATION_RES_14BIT << HDC1008_CONFIGURATION_TRES_SHIFT)
#  define HDC1008_CONFIGURATION_TRES_11BIT    (CONFIGURATION_RES_11BIT << HDC1008_CONFIGURATION_TRES_SHIFT)
#define HDC1008_CONFIGURATION_BTST            (1 << 11) /* Bit 11: Battery status */
#define HDC1008_CONFIGURATION_MODE            (1 << 12) /* Bit 12: Mode of acquisition */
#define HDC1008_CONFIGURATION_HEAT_SHIFT      (13)      /* Bit 13: Heater */
#define HDC1008_CONFIGURATION_HEAT_MASK       (0x01 << HDC1008_CONFIGURATION_HEAT_SHIFT)
#  define HDC1008_CONFIGURATION_HEAT_DISABLE  (0x00 << HDC1008_CONFIGURATION_HEAT_SHIFT)
#  define HDC1008_CONFIGURATION_HEAT_ENABLE   (0x01 << HDC1008_CONFIGURATION_HEAT_SHIFT)
                                                        /* Bit 14: Reserved */
#define HDC1008_CONFIGURATION_RST             (1 << 15) /* Bit 15: Software reset bit */

#endif /* __DRIVERS_SENSORS_HDC1008_BASE_H */
