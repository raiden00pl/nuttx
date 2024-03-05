/****************************************************************************
 * arch/risc-v/src/common/riscv_addrenv_pgmap.c
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

#include <nuttx/arch.h>
#include <nuttx/addrenv.h>
#include <nuttx/irq.h>
#include <nuttx/pgalloc.h>
#include <nuttx/sched.h>

#include <sys/mman.h>

#ifdef CONFIG_BUILD_KERNEL

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_MM_KMAP
static struct arch_addrenv_s g_kernel_addrenv;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_addrenv_find_page
 *
 * Description:
 *   Find physical page mapped to user virtual address from the address
 *   environment page directory.
 *
 * Input Parameters:
 *   addrenv - The user address environment.
 *   vaddr   - The user virtual address
 *
 * Returned Value:
 *   Page physical address on success; NULL on failure.
 *
 ****************************************************************************/

uintptr_t up_addrenv_find_page(arch_addrenv_t *addrenv, uintptr_t vaddr)
{
#warning missing logic: up_addrenv_find_page
  ASSERT(0);
}

/****************************************************************************
 * Name: up_addrenv_page_vaddr
 *
 * Description:
 *   Find the kernel virtual address associated with physical page.
 *
 * Input Parameters:
 *   page - The page physical address.
 *
 * Returned Value:
 *   Page kernel virtual address on success; NULL on failure.
 *
 ****************************************************************************/

uintptr_t up_addrenv_page_vaddr(uintptr_t page)
{
#warning missing logic: up_addrenv_page_vaddr
  ASSERT(0);
}

/****************************************************************************
 * Name: up_addrenv_user_vaddr
 *
 * Description:
 *   Check if a virtual address is in user virtual address space.
 *
 * Input Parameters:
 *   vaddr - The virtual address.
 *
 * Returned Value:
 *   True if it is; false if it's not
 *
 ****************************************************************************/

bool up_addrenv_user_vaddr(uintptr_t vaddr)
{
#warning missing logic: up_addrenv_user_vaddr
  ASSERT(0);
}

/****************************************************************************
 * Name: up_addrenv_page_wipe
 *
 * Description:
 *   Wipe a page of physical memory, first mapping it into kernel virtual
 *   memory.
 *
 * Input Parameters:
 *   page - The page physical address.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void up_addrenv_page_wipe(uintptr_t page)
{
#warning missing logic: up_addrenv_page_wipe
  ASSERT(0);
}

#ifdef CONFIG_MM_KMAP

/****************************************************************************
 * Name: up_addrenv_kmap_init
 *
 * Description:
 *   Initialize the architecture specific part of the kernel mapping
 *   interface.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int up_addrenv_kmap_init(void)
{
#warning missing logic: up_addrenv_kmap_init
  ASSERT(0);
}

/****************************************************************************
 * Name: up_addrenv_kmap_pages
 *
 * Description:
 *   Map physical pages into a continuous virtual memory block.
 *
 * Input Parameters:
 *   pages - A pointer to the first element in a array of physical address,
 *     each corresponding to one page of memory.
 *   npages - The number of pages in the list of physical pages to be mapped.
 *   vaddr - The virtual address corresponding to the beginning of the
 *     (continuous) virtual address region.
 *   prot - Access right flags.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int up_addrenv_kmap_pages(void **pages, unsigned int npages, uintptr_t vaddr,
                          int prot)
{
#warning missing logic: up_addrenv_kmap_pages
  ASSERT(0);
}

/****************************************************************************
 * Name: up_addrenv_kunmap_pages
 *
 * Description:
 *   Unmap a previously mapped virtual memory region.
 *
 * Input Parameters:
 *   vaddr - The virtual address corresponding to the beginning of the
 *     (continuous) virtual address region.
 *   npages - The number of pages to be unmapped
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int up_addrenv_kunmap_pages(uintptr_t vaddr, unsigned int npages)
{
#warning missing logic: up_addrenv_kunmap_pages
  ASSERT(0);
}

#endif /* CONFIG_MM_KMAP */
#endif /* CONFIG_BUILD_KERNEL */
