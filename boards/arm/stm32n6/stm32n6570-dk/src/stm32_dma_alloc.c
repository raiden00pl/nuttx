/****************************************************************************
 * boards/arm/stm32n6/stm32n6570-dk/src/stm32_dma_alloc.c
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
#include <syslog.h>
#include <stdint.h>
#include <errno.h>
#include <nuttx/mm/gran.h>

#include "chip.h"
#include "stm32n6570-dk.h"

#if defined(CONFIG_FAT_DMAMEMORY)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_GRAN)
#  error microSD DMA support requires CONFIG_GRAN
#endif

#define BOARD_DMA_ALLOC_POOL_SIZE (8*512)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static GRAN_HANDLE dma_allocator;

/* The DMA heap size constrains the total number of things that can be
 * ready to do DMA at a time.
 *
 * For example, FAT DMA depends on one sector-sized buffer per
 * filesystem plus one sector-sized buffer per file.
 *
 * With the D-cache, SDMMC IDMA buffers must start and end on a 32 B cache
 * line, so the granule and the alignment are one cache line.
 */

static uint8_t g_dma_heap[BOARD_DMA_ALLOC_POOL_SIZE]
                aligned_data(ARMV8M_DCACHE_LINESIZE);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dma_alloc_init
 *
 * Description:
 *   All boards may optionally provide this API to instantiate a pool of
 *   memory for uses with FAST FS DMA operations.
 *
 ****************************************************************************/

int stm32_dma_alloc_init(void)
{
  dma_allocator = gran_initialize(g_dma_heap,
                                  sizeof(g_dma_heap),
                                  5,  /* 32 B granule */
                                  5); /* 32 B alignment */

  if (dma_allocator == NULL)
    {
      return -ENOMEM;
    }

  return OK;
}

/* DMA-aware allocator stubs for the FAT filesystem. */

void *fat_dma_alloc(size_t size)
{
  return gran_alloc(dma_allocator, size);
}

void fat_dma_free(void *memory, size_t size)
{
  gran_free(dma_allocator, memory, size);
}

#endif /* CONFIG_FAT_DMAMEMORY */
