/****************************************************************************
 * drivers/sensors/lps22hb.c
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
/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#define LPS22HB_I2C_FREQUENCY (400000)

/* Registers */

#define LPS22HB_INTERRUPT_CFG (0x0b)
#define LPS22HB_THS_P_L       (0x0c)
#define LPS22HB_THS_P_H       (0x0d)
#define LPS22HB_WHO_AM_I      (0x0f)
#define LPS22HB_CTRL_REG1     (0x10)
#define LPS22HB_CTRL_REG2     (0x11)
#define LPS22HB_CTRL_REG3     (0x12)
#define LPS22HB_FIFO_CTRL     (0x14)
#define LPS22HB_REF_P_XL      (0x15)
#define LPS22HB_REF_P_L       (0x16)
#define LPS22HB_REF_P_H       (0x17)
#define LPS22HB_RPDS_L        (0x18)
#define LPS22HB_RPDS_H        (0x19)
#define LPS22HB_RES_CONF      (0x1a)
#define LPS22HB_INT_SOURCE    (0x25)
#define LPS22HB_FIFO_STATUS   (0x26)
#define LPS22HB_STATUS        (0x27)
#define LPS22HB_PRESS_OUT_XL  (0x28)
#define LPS22HB_PRESS_OUT_L   (0x29)
#define LPS22HB_PRESS_OUT_H   (0x2a)
#define LPS22HB_TEMP_OUT_L    (0x2b)
#define LPS22HB_TEMP_OUT_H    (0x2c)
#define LPS22HB_LPFP_RES      (0x33)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct lps25h_dev_s
{
  struct i2c_master_s *i2c;
  uint8_t addr;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
