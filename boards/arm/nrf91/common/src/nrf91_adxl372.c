/****************************************************************************
 * boards/arm/nrf91/common/src/nrf91_adxl372.c
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

#include <debug.h>
#include <errno.h>
#include <stdio.h>

#include <nuttx/spi/spi.h>
#include <nuttx/sensors/adxl372.h>

#include "nrf91_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf91_adxl372_init
 ****************************************************************************/

int nrf91_adxl372_init(int devno, int busno, uint8_t addr)
{
  struct spi_dev_s *spi;
  int ret;

  UNUSED(addr);

  sninfo("Initializing ADXL372!\n");

  /* Initialize SPI */

  spi = nrf91_spibus_initialize(busno);
  if (!spi)
    {
      return -ENODEV;
    }

  /* Then register the barometer sensor */

  ret = adxl372_register_uorb(devno, spi);
  if (ret < 0)
    {
      snerr("ERROR: Error registering ADXL372\n");
    }

  return ret;
}
