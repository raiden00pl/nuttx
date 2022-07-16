/****************************************************************************
 * include/nuttx/sensors/lps22h.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_LPS22H_H
#define __INCLUDE_NUTTX_SENSORS_LPS22H_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LPS22H_CMD_INTERRUPT_CFG 0x0b
#define LPS22H_CMD_THS_P_L       0x0c
#define LPS22H_CMD_THS_P_H       0x0d
#define LPS22H_CMD_WHO_AM_I      0x0f
#define LPS22H_CMD_CTRL_REG1     0x10
#define LPS22H_CMD_CTRL_REG2     0x11
#define LPS22H_CMD_CTRL_REG3     0x12
#define LPS22H_CMD_FIFO_CTRL     0x14
#define LPS22H_CMD_REF_P_XL      0x15
#define LPS22H_CMD_REF_P_L       0x16
#define LPS22H_CMD_REF_P_H       0x17
#define LPS22H_CMD_RPDS_L        0x18
#define LPS22H_CMD_RPDS_H        0x19
#define LPS22H_CMD_RES_CONF      0x1a
#define LPS22H_CMD_INT_SOURCE    0x25
#define LPS22H_CMD_FIFO_STATUS   0x26
#define LPS22H_CMD_STATUS        0x27
#define LPS22H_CMD_PRESS_OUT_XL  0x28
#define LPS22H_CMD_PRESS_OUT_L   0x29
#define LPS22H_CMD_PRESS_OUT_H   0x2a
#define LPS22H_CMD_TEMP_OUT_L    0x2b
#define LPS22H_CMD_TEMP_OUT_H    0x2c
#define LPS22H_CMD_LPFP_RES      0x33

#define WHO_AM_I_DEFAULT         0xb1
#define CTRL_REG1_3WIRE          0x01

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct lps22h_measure_s
{
  int32_t temperature;  /* in Degree   x100    */
  int32_t pressure;     /* in mBar     x10     */

};

struct i2c_master_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: lps22h_register
 *
 * Description:
 *   Register the LPS22H character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/press0"
 *   dev     - An instance of the I2C/SPI interface to use to communicate with
 *             LPS22H
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_SENSORS_LPS22H_I2C
int lps22h_register(FAR const char *devpath, FAR struct i2c_master_s *dev);
#else /* CONFIG_SENSORS_LPS22H_SPI */
int lps22h_register(FAR const char *devpath, FAR struct spi_dev_s *dev);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_SENSORS_LPS22H_H */
