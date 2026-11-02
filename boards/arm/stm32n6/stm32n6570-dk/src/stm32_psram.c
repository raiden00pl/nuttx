/****************************************************************************
 * boards/arm/stm32n6/stm32n6570-dk/src/stm32_psram.c
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
#include <stdbool.h>
#include <stdint.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/cache.h>
#include <nuttx/kmalloc.h>

#include <arch/board/board.h>

#include "stm32.h"
#include "stm32_xspi.h"
#include "stm32n6570-dk.h"

#ifdef CONFIG_STM32N6570_DK_PSRAM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_STM32N6570_DK_PSRAM_HEAP) && CONFIG_MM_REGIONS < 2
#  error "The PSRAM heap needs CONFIG_MM_REGIONS >= 2"
#endif

/* APS256XXN-OBR: 256 Mbit, mapped by XSPI1 at 0x90000000 */

#define PSRAM_BASE              STM32_XSPI1_BANK
#define PSRAM_SIZE              (32 * 1024 * 1024)
#define PSRAM_DEVSIZE           24          /* 2^(24 + 1) bytes */

/* XSPI clock: kernel clock / 4 while the mode registers are written, then
 * at most PSRAM_FREQUENCY (100 MHz with the 200 MHz HCLK kernel clock).
 */

#define PSRAM_INIT_PRESCALER    3
#define PSRAM_FREQUENCY         100000000
#define PSRAM_PRESCALER \
  ((STM32_XSPI1_FREQUENCY + PSRAM_FREQUENCY - 1) / PSRAM_FREQUENCY)
#define PSRAM_BUS_FREQUENCY     (STM32_XSPI1_FREQUENCY / PSRAM_PRESCALER)

/* Refresh: release nCS at least every 2 us (tCEM is 4 us) */

#define PSRAM_REFRESH           (2 * (PSRAM_BUS_FREQUENCY / 1000000) - 4)

/* Do not cross a 2 KiB page (row) within one chip-select cycle */

#define PSRAM_CSBOUND           11

/* Commands */

#define APS_READ_LINEAR         0x20        /* Linear burst read */
#define APS_WRITE_LINEAR        0xa0        /* Linear burst write */
#define APS_READ_REG            0x40        /* Mode register read */
#define APS_WRITE_REG           0xc0        /* Mode register write */
#define APS_RESET               0xff        /* Global reset */

/* Mode registers, values as programmed by the ST BSP */

#define APS_MR0                 0           /* Latency type, read latency */
#define APS_MR0_VALUE           0x30        /* Fixed latency, RL code 7 */
#define APS_MR1                 1           /* Vendor ID */
#define APS_MR1_VENDOR_MASK     0x1f
#define APS_MR1_VENDOR_APM      0x0d        /* APMemory */
#define APS_MR4                 4           /* Write latency */
#define APS_MR4_VALUE           0x20        /* WL code 7 */
#define APS_MR8                 8           /* Burst and I/O width */
#define APS_MR8_VALUE           0x40        /* x16 I/O, 16-byte wrap */

/* Latency code 7 (up to 200 MHz) gives 6 dummy cycles */

#define APS_LATENCY_DUMMY       6

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psram_xfer
 *
 * Description:
 *   Fill an XSPI transfer for an octal APMemory command: 8-bit instruction
 *   in STR, 32-bit address and data in DTR.
 *
 ****************************************************************************/

static void psram_xfer(struct stm32_xspi_xfer_s *xfer, uint8_t instruction,
                       uint32_t address, void *buffer, size_t buflen,
                       uint8_t data_lines, uint8_t dummy, bool write)
{
  xfer->instruction       = instruction;
  xfer->address           = address;
  xfer->buffer            = buffer;
  xfer->buflen            = buflen;
  xfer->instruction_lines = 8;
  xfer->address_lines     = 8;
  xfer->data_lines        = data_lines;
  xfer->instruction_bits  = 8;
  xfer->address_bits      = 32;
  xfer->dummy_cycles      = dummy;
  xfer->instruction_dtr   = false;
  xfer->address_dtr       = true;
  xfer->data_dtr          = true;
  xfer->dqs               = dummy != 0;
  xfer->dhqc              = true;
  xfer->write             = write;
}

/****************************************************************************
 * Name: psram_writereg
 ****************************************************************************/

static int psram_writereg(struct qspi_dev_s *qspi, uint32_t reg,
                          uint8_t value)
{
  struct stm32_xspi_xfer_s xfer;
  uint8_t data[2];

  /* A register write carries one DTR word: both bytes hold the value */

  data[0] = value;
  data[1] = value;
  psram_xfer(&xfer, APS_WRITE_REG, reg, data, sizeof(data), 8, 0, true);
  return stm32_xspi_transfer(qspi, &xfer);
}

/****************************************************************************
 * Name: psram_readreg
 ****************************************************************************/

static int psram_readreg(struct qspi_dev_s *qspi, uint32_t reg,
                         uint8_t *value)
{
  struct stm32_xspi_xfer_s xfer;
  uint8_t data[2];
  int ret;

  /* A DTR register read returns the 16-bit pair of mode registers at the
   * even address: MRn first, then MRn+1.
   */

  psram_xfer(&xfer, APS_READ_REG, reg & ~1, data, sizeof(data), 8,
             APS_LATENCY_DUMMY, false);
  ret = stm32_xspi_transfer(qspi, &xfer);
  *value = data[reg & 1];
  return ret;
}

/****************************************************************************
 * Name: psram_reset
 ****************************************************************************/

static int psram_reset(struct qspi_dev_s *qspi)
{
  struct stm32_xspi_xfer_s xfer;
  int ret;

  psram_xfer(&xfer, APS_RESET, 0, NULL, 0, 0, 0, true);
  xfer.address_bits = 24;
  xfer.address_dtr  = false;
  xfer.data_dtr     = false;

  ret = stm32_xspi_transfer(qspi, &xfer);

  /* Wait for the reset to complete, as the ST BSP does */

  up_mdelay(1);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_psram_initialize
 *
 * Description:
 *   Initialize XSPI1 and the APS256XXN PSRAM, map it at 0x90000000 and,
 *   with CONFIG_STM32N6570_DK_PSRAM_HEAP, add it to the heap.
 *
 ****************************************************************************/

int stm32_psram_initialize(void)
{
  struct stm32_xspi_config_s config;
  struct stm32_xspi_xfer_s readxfer;
  struct stm32_xspi_xfer_s writexfer;
  struct qspi_dev_s *qspi;
  uint8_t vendor = 0;
  int ret;

  config.memtype      = STM32_XSPI_MEMTYPE_APMEM_16BIT;
  config.iomport      = STM32_XSPI_IOM_PORT1;
  config.cs_override  = STM32_XSPI_CS_DISABLED;
  config.devsize      = PSRAM_DEVSIZE;
  config.csht         = 5;
  config.prescaler    = PSRAM_INIT_PRESCALER;
  config.fthres       = 8;
  config.wrapsize     = 0;
  config.maxtran      = 0;
  config.csbound      = PSRAM_CSBOUND;
  config.req2ack      = 1;
  config.refresh      = PSRAM_REFRESH;
  config.clock_mode3  = false;
  config.free_running = false;
  config.sample_shift = false;

  qspi = stm32_xspi_initialize(1, &config);
  if (qspi == NULL)
    {
      return -ENODEV;
    }

  ret = psram_reset(qspi);
  if (ret == OK)
    {
      ret = psram_writereg(qspi, APS_MR0, APS_MR0_VALUE);
    }

  if (ret == OK)
    {
      ret = psram_writereg(qspi, APS_MR4, APS_MR4_VALUE);
    }

  if (ret == OK)
    {
      ret = psram_readreg(qspi, APS_MR1, &vendor);
    }

  if (ret < 0)
    {
      return ret;
    }

  if ((vendor & APS_MR1_VENDOR_MASK) != APS_MR1_VENDOR_APM)
    {
      syslog(LOG_ERR, "ERROR: PSRAM vendor ID 0x%02x\n", vendor);
      return -ENODEV;
    }

  /* Switch to x16 I/O; register reads are not used beyond this point */

  ret = psram_writereg(qspi, APS_MR8, APS_MR8_VALUE);
  if (ret < 0)
    {
      return ret;
    }

  QSPI_SETFREQUENCY(qspi, PSRAM_FREQUENCY);

  psram_xfer(&readxfer, APS_READ_LINEAR, 0, NULL, 0, 16,
             APS_LATENCY_DUMMY, false);
  psram_xfer(&writexfer, APS_WRITE_LINEAR, 0, NULL, 0, 16,
             APS_LATENCY_DUMMY, true);

  ret = stm32_xspi_memorymap(qspi, &readxfer, &writexfer);
  if (ret < 0)
    {
      return ret;
    }

  /* Nothing may be cached from the window before it was mapped */

  up_invalidate_dcache(PSRAM_BASE, PSRAM_BASE + PSRAM_SIZE);

#ifdef CONFIG_STM32N6570_DK_PSRAM_HEAP
  kumm_addregion((void *)PSRAM_BASE, PSRAM_SIZE);
#endif

  syslog(LOG_INFO, "PSRAM: %d MiB at 0x%08x\n", PSRAM_SIZE >> 20,
         PSRAM_BASE);
  return OK;
}

#endif /* CONFIG_STM32N6570_DK_PSRAM */
