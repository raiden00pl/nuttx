/****************************************************************************
 * boards/arm/nrf91/thingy53/src/nrf91_spi.c
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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>

#include "arm_internal.h"
#include "chip.h"
#include "nrf91_gpio.h"
#include "nrf91_spi.h"

#include "thingy91.h"
#include <arch/board/board.h>

#ifdef CONFIG_NRF91_SPI_MASTER

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf91_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Nucleo-144 board.
 *
 ****************************************************************************/

void nrf91_spidev_initialize(void)
{
#ifdef CONFIG_NRF91_SPI1_MASTER
  nrf91_gpio_config(GPIO_ADXL372_CS);
  nrf91_gpio_write(GPIO_ADXL372_CS, true);

  nrf91_gpio_config(GPIO_ADXL362_CS);
  nrf91_gpio_write(GPIO_ADXL362_CS, true);
#endif
}

/****************************************************************************
 * Name: nrf91_spi0/1/2/3/select and nrf91_spi0/1/2/3/status
 *
 * Description:
 *   The external functions, nrf91_spi0/1/2/3select and
 *   nrf91_spi0/1/2/3status must be provided by board-specific logic.
 *   They are implementations of the select and status methods of the SPI
 *   interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *   All other methods (including nrf91_spibus_initialize()) are provided
 *   by common NRF91 logic.  To use this common SPI logic on your board:
 *
 *   1. Provide logic in nrf91_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide nrf91_spi0/1/2/3select() and nrf91_spi0/1/2/3status()
 *      functions in your board-specific logic. These functions will perform
 *      chip selection and status operations using GPIOs in the way your
 *      board is configured.
 *   3. Add a calls to nrf91_spibus_initialize() in your low level
 *      application initialization logic
 *   4. The handle returned by nrf91_spibus_initialize() may then be used to
 *      bind the SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_NRF91_SPI0_MASTER
void nrf91_spi0select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %08lx CS: %s\n",
          (unsigned long)devid, selected ? "assert" : "de-assert");
}

uint8_t nrf91_spi0status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_NRF91_SPI1_MASTER
void nrf91_spi1select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %08lx CS: %s\n", (unsigned long)devid,
          selected ? "assert" : "de-assert");

  switch (devid)
    {
#ifdef CONFIG_SENSORS_ADXL372
      case SPIDEV_ACCELEROMETER(ADXL372_SPI_DEVNO):
        {
          spiinfo("ADXL372 device %s\n",
                  selected ? "asserted" : "de-asserted");

          /* Set the GPIO low to select and high to de-select */

          nrf91_gpio_write(GPIO_ADXL372_CS, !selected);
          break;
        }
#endif

#ifdef CONFIG_SENSORS_ADXL362
      case SPIDEV_ACCELEROMETER(ADXL362_SPI_DEVNO):
        {
          spiinfo("ADXL372 device %s\n",
                  selected ? "asserted" : "de-asserted");

          /* Set the GPIO low to select and high to de-select */

          nrf91_gpio_write(GPIO_ADXL362_CS, !selected);
          break;
        }
#endif

      default:
        {
          break;
        }
    }
}

uint8_t nrf91_spi1status(struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t status = 0;

  switch (devid)
    {
#ifdef CONFIG_SENSORS_ADXL372
      case SPIDEV_ACCELEROMETER(ADXL372_SPI_DEVNO):
        {
          status |= SPI_STATUS_PRESENT;
          break;
        }
#endif

#ifdef CONFIG_SENSORS_ADXL362
      case SPIDEV_ACCELEROMETER(ADXL362_SPI_DEVNO):
        {
          status |= SPI_STATUS_PRESENT;
          break;
        }
#endif

      default:
        {
          break;
        }
    }

  return status;
}
#endif

#ifdef CONFIG_nrf91_SPI2_MASTER
void nrf91_spi2select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %08lx CS: %s\n",
          (unsigned long)devid, selected ? "assert" : "de-assert");
}

uint8_t nrf91_spi2status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#ifdef CONFIG_NRF91_SPI3_MASTER
void nrf91_spi3select(struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %08lx CS: %s\n",
          (unsigned long)devid, selected ? "assert" : "de-assert");
}

uint8_t nrf91_spi3status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif

#endif /* CONFIG_NRF91_SPI_MASTER */
