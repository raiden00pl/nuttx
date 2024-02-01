/*****************************************************************************
 * drivers/virt/ivshmem.c
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
 *****************************************************************************/

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <debug.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/pci/pci.h>

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

#define IVSHMEM_BAR_REG   (0)
#define IVSHMEM_BAR_MSIX  (1)
#define IVSHMEM_BAR_SHMEM (2)

/*****************************************************************************
 * Private Functions Definitions
 *****************************************************************************/

/* Registers */

struct ivshmem_reg_s
{
	uint32_t int_mask;    /* Interrupt Mask register (legacy) */
	uint32_t int_status;  /* Interrupt Status register (legacy) */
	uint32_t iv_position; /* Inter-VM position register */
	uint32_t doorbell;    /* Doorbel register */
};

struct ivshmem_s
{
  uintptr_t shmem;
  uint64_t shmem_size;
};

/*****************************************************************************
 * Private Functions Definitions
 *****************************************************************************/

static int ivshmem_probe(FAR struct pci_bus_s *bus,
                         FAR const struct pci_dev_type_s *type,
                         uint16_t bdf);

/*****************************************************************************
 * Private Types
 *****************************************************************************/

/*****************************************************************************
 * Public Data
 *****************************************************************************/

const struct pci_dev_type_s g_pci_type_ivshmem =
{
  .vendor    = 0x1af4,
  .device    = 0x1110,
  .class_rev = PCI_ID_ANY,
  .name      = "Inter-VM shared memory",
  .probe     = ivshmem_probe
};

static struct ivshmem_s g_ivshmem;

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: ivshmem_probe
 *
 * Description:
 *   Initialize device
 *
 *****************************************************************************/

static int ivshmem_probe(FAR struct pci_bus_s *bus,
                         FAR const struct pci_dev_type_s *type,
                         uint16_t bdf)
{
  struct pci_dev_s          dev;
  FAR struct ivshmem_reg_s *regs      = NULL;
  uintptr_t                 bar_reg   = 0;
  uintptr_t                 bar_shmem = 0;
  uintptr_t                 bar_msix  = 0;
  bool                      no_msix   = false;

  /* Get dev */

  dev.bus = bus;
  dev.type = type;
  dev.bdf = bdf;

  pci_enable_bus_master(&dev);
  pci_enable_io(&dev, PCI_SYS_RES_MEM);

  /* Get registers BAR */

  if (pci_bar_valid(&dev, IVSHMEM_BAR_REG) != OK)
    {
      pcierr("Registers BAR is not valid\n");
      return -EINVAL;
    }

  bar_reg = pci_bar_addr(&dev, IVSHMEM_BAR_REG);
  if (bus->ops->pci_map_bar(bar_regs,
                            pci_bar_size(&dev, EDU_CONTROL_BAR_ID)) != OK)
    {
      pcierr("Failed to map registers BAR\n");
      return -EINVAL;
    }

  regs = (struct ivshmem_reg_s *)bar_reg;

  /* Get MSI-X BAR */

  if (pci_bar_valid(&dev, IVSHMEM_BAR_MSIX) != OK)
    {
      pcierr("MSI-X BAR is not valid\n");
      no_msix = true;
    }
  else
    {
      bar_msix = pci_bar_addr(&dev, IVSHMEM_BAR_MSIX);
      if (bus->ops->pci_map_bar(bar_msix,
                                pci_bar_size(&dev, EDU_CONTROL_BAR_ID)) != OK)
        {
          pcierr("Failed to map BAR_MSIX\n");
          return -EINVAL;
        }
    }

  /* Get SHMEM BAR */

  if (pci_bar_valid(&dev, IVSHMEM_BAR_SHMEM) != OK)
    {
      pcierr("SHMEM BAR is not valid\n");
      return -EINVAL;
    }

  bar_shmem = pci_bar_addr(&dev, IVSHMEM_BAR_SHMEM);
  shmem_size = pci_bar_size(&dev, EDU_CONTROL_BAR_ID);
  if (bus->ops->pci_map_bar(bar_shmem, shmem_size) != OK)
    {
      pcierr("Failed to map BAR_SHMEM\n");
      return -EINVAL;
    }

  /* Get SHMEM */

  g_ivshmem.shmem = bar_shmem;
  g_ivshmem.shmem_size = shmem_size;


  /* Configure MSI-X vectors */

  int i = 0;
  for (i = i; < VIRT_IVSHMEM_DOORBEL_VECTORS; i += 1)
    {


    }

  return OK;
}
