/****************************************************************************
 * boards/arm/stm32n6/stm32n6570-dk/src/stm32_mx66uw1g45g.c
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
#include <syslog.h>

#include <nuttx/fs/fs.h>
#include <nuttx/mtd/mtd.h>

#include "stm32_xspi.h"
#include "stm32n6570-dk.h"

#ifdef CONFIG_STM32N6570_DK_NOR

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MX66UW1G45G: 128 MiB of 256-byte pages.  The boot image lives at the
 * start of the flash, so only the part from CONFIG_STM32N6570_DK_NOR_OFFSET
 * up is exposed.
 */

#define NOR_SIZE          (128 * 1024 * 1024)
#define NOR_DEVSIZE       26                 /* 2^(26 + 1) bytes */
#define NOR_PAGE_SIZE     256

#if CONFIG_STM32N6570_DK_NOR_OFFSET < 0x100000 || \
    CONFIG_STM32N6570_DK_NOR_OFFSET >= NOR_SIZE || \
    (CONFIG_STM32N6570_DK_NOR_OFFSET & 0xffff) != 0
#  error "Invalid CONFIG_STM32N6570_DK_NOR_OFFSET"
#endif

#define NOR_PART_FIRST    (CONFIG_STM32N6570_DK_NOR_OFFSET / NOR_PAGE_SIZE)
#define NOR_PART_NBLOCKS  \
  ((NOR_SIZE - CONFIG_STM32N6570_DK_NOR_OFFSET) / NOR_PAGE_SIZE)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_nor_initialize
 *
 * Description:
 *   Initialize XSPI2 and the MX66UW1G45G NOR flash and register the
 *   partition above CONFIG_STM32N6570_DK_NOR_OFFSET as /dev/nor.  With
 *   CONFIG_FS_LITTLEFS it is mounted at /data (formatted when empty).
 *
 ****************************************************************************/

int stm32_nor_initialize(void)
{
  struct stm32_xspi_config_s config;
  struct qspi_dev_s *qspi;
  struct mtd_dev_s *mtd;
  struct mtd_dev_s *part;
  int ret;

  config.memtype      = STM32_XSPI_MEMTYPE_MACRONIX;
  config.iomport      = STM32_XSPI_IOM_PORT2;
  config.cs_override  = STM32_XSPI_CS_DISABLED;
  config.devsize      = NOR_DEVSIZE;
  config.csht         = 2;
  config.prescaler    = 3;
  config.fthres       = 4;
  config.wrapsize     = 0;
  config.maxtran      = 0;
  config.csbound      = 0;
  config.req2ack      = 1;
  config.refresh      = 0;
  config.clock_mode3  = false;
  config.free_running = false;
  config.sample_shift = false;

  qspi = stm32_xspi_initialize(2, &config);
  if (qspi == NULL)
    {
      return -ENODEV;
    }

  /* The MTD driver resets the flash and switches it to octal STR */

  mtd = mx66uw1g45g_initialize(qspi);
  if (mtd == NULL)
    {
      return -ENODEV;
    }

  part = mtd_partition(mtd, NOR_PART_FIRST, NOR_PART_NBLOCKS);
  if (part == NULL)
    {
      return -ENOMEM;
    }

  ret = register_mtddriver("/dev/nor", part, 0755, NULL);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_FS_LITTLEFS
  ret = nx_mount("/dev/nor", "/data", "littlefs", 0, "autoformat");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: littlefs mount on /data failed: %d\n", ret);
      return ret;
    }
#endif

  return OK;
}

#endif /* CONFIG_STM32N6570_DK_NOR */
