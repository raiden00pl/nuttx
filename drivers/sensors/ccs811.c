/****************************************************************************
 * drivers/sensors/ccs811.c
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

#error

#include <nuttx/sensors/ccs811.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Registers */

#define CCS811_STATUS          (0x00)
#define CCS811_MEAS_MODE       (0x00)
#define CCS811_ALG_RESULT_DATA (0x02)
#define CCS811_RAW_DATA        (0x03)
#define CCS811_ENV_DATA        (0x05)
#define CCS811_THRESHOLDS      (0x10)
#define CCS811_BASELINE        (0x11)
#define CCS811_HW_ID           (0x20)
#define CCS811_HW_VERSION      (0x21)
#define CCS811_FW_BOOT_VERSION (0x23)
#define CCS811_FW_APP_VERSION  (0x24)
#define CCS811_INTERNAL_STATE  (0xa0)
#define CCS811_ERROR_ID        (0xe0)
#define CCS811_SW_RESET        (0xff)
#define CCS811_APP_ERASE       (0xf1)
#define CCS811_APP_DATA        (0xf2)
#define CCS811_APP_VERIFY      (0xf3)
#define CCS811_APP_START       (0xf4)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
