/****************************************************************************
 * boards/arm/stm32n6/stm32n6570-dk/src/stm32_camera.c
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

#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/video/imgdata.h>
#include <nuttx/video/imgsensor.h>
#include <nuttx/video/imx335.h>

#include "stm32_gpio.h"
#include "stm32_i2c.h"
#include "stm32_dcmipp.h"
#include "stm32n6570-dk.h"

#if defined(CONFIG_STM32_DCMIPP) && defined(CONFIG_VIDEO_IMX335)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The MB1854 module has its own 24 MHz oscillator for the IMX335 */

#define CAMERA_INCK  24000000

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* IMX335: 2592x1944 RAW10 RGGB on two CSI-2 lanes.  The D-PHY range, the
 * black level, the white balance gains and the colour matrix are the
 * ST values for this module (fluorescent TL84 light).
 */

static const struct stm32_dcmipp_config_s g_camera_config =
{
  .width      = 2592,
  .height     = 1944,
  .lanes      = 2,
  .bitrate    = 1600,
  .bpp        = 10,
  .bayer      = STM32_DCMIPP_BAYER_RGGB,
  .blacklevel = 12,
  .gain       =
    {
      233, 128, 271
    },
  .ccm        =
    {
      {
        422, -54, -112
      },
      {
        -131, 457, -70
      },
      {
        -32, -123, 411
      },
    },
  .gamma      = true,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_camera_initialize
 ****************************************************************************/

int stm32_camera_initialize(void)
{
  FAR struct i2c_master_s *i2c;
  FAR struct imgsensor_s *sensor;
  FAR struct imgdata_s *data;

  /* Power the module with the sensor held in reset, then release it */

  stm32_configgpio(GPIO_CAM_NRST);
  stm32_configgpio(GPIO_CAM_EN);
  nxsig_usleep(100000);

  stm32_gpiowrite(GPIO_CAM_EN, true);
  nxsig_usleep(10000);
  stm32_gpiowrite(GPIO_CAM_NRST, true);
  nxsig_usleep(20000);

  i2c = stm32_i2cbus_initialize(CAMERA_I2C_BUS);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  sensor = imx335_initialize(i2c, CAMERA_INCK);
  if (sensor == NULL)
    {
      return -ENODEV;
    }

  data = stm32_dcmipp_initialize(&g_camera_config);
  if (data == NULL)
    {
      return -ENODEV;
    }

  imgdata_register(data);
  return imgsensor_register(sensor);
}

#endif /* CONFIG_STM32_DCMIPP && CONFIG_VIDEO_IMX335 */
