/****************************************************************************
 * arch/arm/src/common/stm32/stm32_xspi.h
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

#ifndef __ARCH_ARM_SRC_COMMON_STM32_STM32_XSPI_H
#define __ARCH_ARM_SRC_COMMON_STM32_STM32_XSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <nuttx/spi/qspi.h>

#ifdef CONFIG_STM32_XSPI

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum stm32_xspi_memtype_e
{
  STM32_XSPI_MEMTYPE_MICRON       = 0,
  STM32_XSPI_MEMTYPE_MACRONIX     = 1,
  STM32_XSPI_MEMTYPE_APMEM        = 2,
  STM32_XSPI_MEMTYPE_MACRONIX_RAM = 3,
  STM32_XSPI_MEMTYPE_HYPERBUS     = 4,
  STM32_XSPI_MEMTYPE_APMEM_16BIT  = 6
};

enum stm32_xspi_iomport_e
{
  STM32_XSPI_IOM_PORT1 = 0,
  STM32_XSPI_IOM_PORT2 = 1
};

enum stm32_xspi_cs_e
{
  STM32_XSPI_CS_DISABLED = 0,
  STM32_XSPI_CS_NCS1,
  STM32_XSPI_CS_NCS2
};

struct stm32_xspi_config_s
{
  enum stm32_xspi_memtype_e memtype;
  enum stm32_xspi_iomport_e iomport;
  enum stm32_xspi_cs_e cs_override;
  uint8_t devsize;
  uint8_t csht;
  uint8_t prescaler;
  uint8_t fthres;
  uint8_t wrapsize;
  uint8_t maxtran;
  uint8_t csbound;
  uint16_t req2ack;
  uint32_t refresh;
  bool clock_mode3;
  bool free_running;
  bool sample_shift;
};

/* One XSPI command described at the register level, for memories that the
 * generic QSPI interface cannot describe (16 data lines, per-phase DTR,
 * DQS), such as APMemory PSRAMs.
 */

struct stm32_xspi_xfer_s
{
  uint32_t instruction;      /* Instruction */
  uint32_t address;          /* Address (indirect mode) */
  void *buffer;              /* Data buffer (indirect mode) */
  size_t buflen;             /* Data length in bytes (indirect mode) */
  uint8_t instruction_lines; /* Instruction lines: 0, 1, 2, 4 or 8 */
  uint8_t address_lines;     /* Address lines: 0, 1, 2, 4 or 8 */
  uint8_t data_lines;        /* Data lines: 0, 1, 2, 4, 8 or 16 */
  uint8_t instruction_bits;  /* Instruction size: 8, 16, 24 or 32 bits */
  uint8_t address_bits;      /* Address size: 8, 16, 24 or 32 bits */
  uint8_t dummy_cycles;      /* Dummy cycles: 0-31 */
  bool instruction_dtr;      /* Instruction in DTR mode */
  bool address_dtr;          /* Address in DTR mode */
  bool data_dtr;             /* Data in DTR mode */
  bool dqs;                  /* Data strobe (read) and data mask (write) */
  bool dhqc;                 /* Delay output data by a quarter cycle */
  bool write;                /* Write (true) or read (false) */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Name: stm32_xspi_hwenable
 *
 * Description:
 *   Family hook called once per XSPI controller before it is first used.
 *   It enables the I/O supply of the XSPIM port, the controller and XSPIM
 *   clocks, the kernel clock source and the I/O compensation cell.
 *
 * Input Parameters:
 *   intf - XSPI controller number (1 or 2)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_xspi_hwenable(int intf);

/****************************************************************************
 * Name: stm32_xspi_initialize
 *
 * Description:
 *   Initialize an XSPI controller.
 *
 * Input Parameters:
 *   intf   - XSPI controller number (1 or 2)
 *   config - XSPI controller configuration
 *
 * Returned Value:
 *   Valid QSPI device structure on success; NULL on failure.
 *
 ****************************************************************************/

struct qspi_dev_s *
stm32_xspi_initialize(int intf, const struct stm32_xspi_config_s *config);

/****************************************************************************
 * Name: stm32_xspi_enter_memorymapped
 *
 * Description:
 *   Configure the XSPI controller for memory-mapped access.
 *
 * Input Parameters:
 *   dev       - QSPI device
 *   readinfo  - Memory transfer parameters used for reading
 *   writeinfo - Memory transfer parameters used for writing; may be NULL
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_xspi_enter_memorymapped(
  struct qspi_dev_s *dev,
  const struct qspi_meminfo_s *readinfo,
  const struct qspi_meminfo_s *writeinfo);

/****************************************************************************
 * Name: stm32_xspi_exit_memorymapped
 *
 * Description:
 *   Take the XSPI device out of memory-mapped mode.
 *
 * Input Parameters:
 *   dev - QSPI device
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_xspi_exit_memorymapped(struct qspi_dev_s *dev);

/****************************************************************************
 * Name: stm32_xspi_transfer
 *
 * Description:
 *   Execute one indirect-mode transfer described at the XSPI register
 *   level.  The caller serializes access with QSPI_LOCK when the device is
 *   shared.
 *
 * Input Parameters:
 *   dev  - QSPI device returned by stm32_xspi_initialize()
 *   xfer - Transfer description
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_xspi_transfer(struct qspi_dev_s *dev,
                        const struct stm32_xspi_xfer_s *xfer);

/****************************************************************************
 * Name: stm32_xspi_memorymap
 *
 * Description:
 *   Enter memory-mapped mode with read and write commands described at the
 *   XSPI register level.
 *
 * Input Parameters:
 *   dev       - QSPI device returned by stm32_xspi_initialize()
 *   readxfer  - Read command; the address, buffer and length are unused
 *   writexfer - Write command; may be NULL
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_xspi_memorymap(struct qspi_dev_s *dev,
                         const struct stm32_xspi_xfer_s *readxfer,
                         const struct stm32_xspi_xfer_s *writexfer);

#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_STM32_XSPI */
#endif /* __ARCH_ARM_SRC_COMMON_STM32_STM32_XSPI_H */
