/****************************************************************************
 * boards/arm/stm32n6/stm32n6570-dk/src/stm32_bringup.c
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

#include <sys/types.h>
#include <syslog.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/leds/userled.h>
#include <nuttx/input/buttons.h>
#include <nuttx/timers/pwm.h>
#include <nuttx/timers/rtc.h>
#include <nuttx/video/fb.h>

#include "stm32_rtc.h"
#include "stm32n6570-dk.h"
#include "board_common.h"

#ifdef CONFIG_INPUT_GT9XX
#  include "stm32_gt9xx.h"
#endif

#include <arch/board/board.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret;

#ifdef CONFIG_STM32N6570_DK_PSRAM
  /* Map the PSRAM first so that later allocations can use it */

  ret = stm32_psram_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_psram_initialize() failed: %d\n", ret);
    }
#endif

  /* Initialize the board features common to all STM32 boards: procfs and
   * the enabled I2C buses.
   */

  ret = stm32_common_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_common_initialize() failed: %d\n",
             ret);
    }

#if !defined(CONFIG_ARCH_LEDS) && defined(CONFIG_USERLED_LOWER)
  /* Register the LED driver */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_INPUT_BUTTONS_LOWER
  /* Register the button driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_STM32_SPI
  /* Initialize the SPI buses and their chip selects */

  stm32_spiinitialize();

#  ifdef CONFIG_SPI_DRIVER
  stm32_spiregister();
#  endif
#endif

#if defined(CONFIG_RTC_DRIVER) && defined(CONFIG_STM32_RTC)
  /* Register the RTC driver as /dev/rtc0 */

  ret = rtc_initialize(0, stm32_rtc_lowerhalf());
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: rtc_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_STM32N6570_DK_NOR
  /* Register the NOR flash data partition and mount it */

  ret = stm32_nor_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_nor_initialize() failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_STM32_SDMMC2) && defined(CONFIG_MMCSD_SDIO)
  /* Initialize the microSD slot */

  ret = stm32_sdio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_sdio_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_PWM
  /* Initialize PWM and register the PWM device */

  ret = stm32_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_pwm_setup() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_VIDEO_FB
  /* Initialize and register the LCD framebuffer driver as /dev/fb0 */

  ret = fb_register(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fb_register() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_INPUT_GT9XX
  /* Register the GT911 touch controller as /dev/input0.  The LCD set-up
   * above has released its reset.
   */

  ret = board_gt9xx_initialize(0, TOUCH_I2C_BUS, TOUCH_I2C_ADDR,
                               GPIO_TOUCH_INT);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_gt9xx_initialize() failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_STM32_OTGHS) && defined(CONFIG_USBHOST)
  /* Start the USB host on the USB1 connector */

  ret = stm32_usbhost_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_usbhost_initialize() failed: %d\n",
             ret);
    }
#endif

#ifdef HAVE_WM8904
  /* Register the WM8904 headphone output as /dev/audio/pcm0 */

  ret = stm32_wm8904_initialize(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_wm8904_initialize() failed: %d\n",
             ret);
    }
#endif

#if defined(CONFIG_STM32_DCMIPP) && defined(CONFIG_VIDEO_IMX335)
  /* Register the IMX335 camera; the application creates /dev/video with
   * capture_initialize().
   */

  ret = stm32_camera_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_camera_initialize() failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_ADC) && defined(CONFIG_STM32_ADC1)
  /* Initialize ADC1 and register it as /dev/adc0 */

  ret = stm32_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_adc_setup() failed: %d\n", ret);
    }
#endif

  UNUSED(ret);
  return OK;
}
