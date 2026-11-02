/****************************************************************************
 * arch/arm/src/stm32n6/stm32n6xx_xspi.c
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

#include <stdbool.h>

#include "arm_internal.h"
#include "stm32_xspi.h"
#include "hardware/stm32n6xxx_rcc.h"

#ifdef CONFIG_STM32_XSPI

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The XSPIM is shared by both controllers and is reset only once */

static bool g_xspim_reset;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_xspi_hwenable
 *
 * Description:
 *   Return an STM32N6 XSPI controller to its reset state.  The bus and
 *   XSPIM clocks are enabled at boot (CONFIG_STM32_XSPIn), the kernel clock
 *   source comes from board.h (STM32_RCC_CCIPR6_XSPInSEL) and the VDDIO2/3
 *   supplies and their compensation cells are set up by the startup code.
 *
 *   The flash boot ROM leaves XSPI2 and the XSPIM configured for its own
 *   boot flash access, so the controller and, on first use, the XSPIM are
 *   reset.  The XSPIM then comes back in the direct mapping: XSPI1 on
 *   port 1 and XSPI2 on port 2.
 *
 ****************************************************************************/

int stm32_xspi_hwenable(int intf)
{
  uint32_t rst = intf == 1 ? RCC_AHB5RSTR_XSPI1RST : RCC_AHB5RSTR_XSPI2RST;

  if (!g_xspim_reset)
    {
      rst |= RCC_AHB5RSTR_XSPIMRST;
      g_xspim_reset = true;
    }

  putreg32(rst, STM32_RCC_AHB5RSTSR);
  (void)getreg32(STM32_RCC_AHB5RSTR);
  putreg32(rst, STM32_RCC_AHB5RSTCR);
  (void)getreg32(STM32_RCC_AHB5RSTR);
  return OK;
}

#endif /* CONFIG_STM32_XSPI */
