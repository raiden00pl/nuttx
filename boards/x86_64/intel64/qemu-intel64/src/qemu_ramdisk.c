/****************************************************************************
 * boards/x86_64/intel64/qemu-intel64/src/qemu_ramdisk.c
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

#include <stdbool.h>
#include <stdio.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/board.h>

#include <arch/board/board_memorymap.h>
#include <nuttx/drivers/ramdisk.h>
#include <sys/boardctl.h>
#include <sys/mount.h>

#include "x86_64_internal.h"
#include "qemu_intel64.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_BUILD_KERNEL
#error "Ramdisk usage is intended to be used with kernel build only"
#endif

#define SECTORSIZE   512
#define NSECTORS(b)  (((b) + SECTORSIZE - 1) / SECTORSIZE)
#define RAMDISK_DEVICE_MINOR 0

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qemu_mount_ramdisk
 *
 * Description:
 *  Mount a ramdisk defined in the ld-kernel.script to /dev/ramX.
 *  The ramdisk is intended to contain a romfs with applications which can
 *  be spawned at runtime.
 *
 * Returned Value:
 *   OK is returned on success.
 *   -ERRORNO is returned on failure.
 *
 ****************************************************************************/

int qemu_mount_ramdisk(void)
{
  struct boardioc_romdisk_s desc;
  uintptr_t                 ramdisk_start;
  size_t                    ramdisk_size;
  int                       ret;

  /* Check if tag is valid */

  if (g_initrd_tag.type != MULTIBOOT_TAG_TYPE_MODULE)
    {
      return -ENODEV;
    }

  ramdisk_start = g_initrd_tag.mod_start;
  ramdisk_size  = g_initrd_tag.mod_end - g_initrd_tag.mod_start;

  /* Map RAMDISK region */

  up_map_region((void *)ramdisk_start, ramdisk_size,
                (X86_PAGE_PRESENT | X86_PAGE_WR | X86_PAGE_NOCACHE));

  /* Register RAMDISK */

  desc.minor    = RAMDISK_DEVICE_MINOR;
  desc.nsectors = NSECTORS((ssize_t)ramdisk_size);
  desc.sectsize = SECTORSIZE;
  desc.image    = ramdisk_start;

  ret = boardctl(BOARDIOC_ROMDISK, (uintptr_t)&desc);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Ramdisk register failed: %s\n", strerror(errno));
      syslog(LOG_ERR, "Ramdisk mountpoint /dev/ram%d\n",
                                          RAMDISK_DEVICE_MINOR);
      syslog(LOG_ERR, "Ramdisk length %u, origin %x\n",
                                          (ssize_t)__ramdisk_size,
                                          (uintptr_t)__ramdisk_start);
    }

  return ret;
}
