
/****************************************************************************
 * boards/arm/stm32l4/steval-stlcs01v1/src/stm32_lps22h.c
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
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <sys/types.h>

#include <nuttx/spi/spi.h>
#include <nuttx/sensors/lps22h.h>

#include <arch/board/board.h>

#include "stm32l4_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SENSORS_LPS22H_SPI
#  error
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_lps22h_initialize
 ****************************************************************************/

int stm32l4_lps22h_initialize(void)
{
  struct spi_dev_s *spi = NULL;
  int ret = OK;

  sninfo("INFO: Initializing LPS22H sensor over SPI\n");

  /* Get SPI bus */

  spi = stm32l4_spibus_initialize(2);
  if (spi == NULL)
    {
      ret = -ENODEV;
      goto errout;
    }

  /* Register LPS22H sensor */

  ret = lps22h_register("/dev/lps22h0", spi);
  if (ret < 0)
    {
      serr("ERROR: Failed to initialize lps22h: %d\n", ret);
    }

errout:
  return ret;
}
