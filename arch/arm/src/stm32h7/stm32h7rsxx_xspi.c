/****************************************************************************
 * arch/arm/src/stm32h7/stm32h7rsxx_xspi.c
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

#include "arm_internal.h"
#include "stm32_xspi.h"
#include "hardware/stm32h7rsxx_pwr.h"
#include "hardware/stm32h7rsxx_rcc.h"
#include "hardware/stm32h7rsxx_sbs.h"

#ifdef CONFIG_STM32_XSPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define XSPI_TIMEOUT 1000000

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xspi_compensation_enable
 *
 * Description:
 *   Enable the XSPI I/O compensation cell and wait until it is ready.
 *
 ****************************************************************************/

static int xspi_compensation_enable(int intf)
{
  uint32_t enable;
  uint32_t ready;
  int timeout;

  modifyreg32(STM32_RCC_APB4ENR, 0, RCC_APB4ENR_SBSEN);
  modifyreg32(STM32_RCC_CR, 0, RCC_CR_CSION);

  for (timeout = XSPI_TIMEOUT; timeout > 0; timeout--)
    {
      if ((getreg32(STM32_RCC_CR) & RCC_CR_CSIRDY) != 0)
        {
          break;
        }
    }

  if (timeout == 0)
    {
      return -ETIMEDOUT;
    }

  if (intf == 1)
    {
      enable = SBS_CCCSR_XSPI1_COMP_EN | SBS_CCCSR_XSPI1_IOHSLV;
      ready  = SBS_CCCSR_XSPI1_COMP_RDY;
    }
  else
    {
      enable = SBS_CCCSR_XSPI2_COMP_EN | SBS_CCCSR_XSPI2_IOHSLV;
      ready  = SBS_CCCSR_XSPI2_COMP_RDY;
    }

  modifyreg32(STM32_SBS_CCCSR, 0, enable);
  for (timeout = XSPI_TIMEOUT; timeout > 0; timeout--)
    {
      if ((getreg32(STM32_SBS_CCCSR) & ready) != 0)
        {
          return OK;
        }
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_xspi_hwenable
 *
 * Description:
 *   Enable the XSPIM port supply, the HCLK kernel clock, the controller and
 *   XSPIM clocks and the I/O compensation cell of an STM32H7RS XSPI.
 *
 ****************************************************************************/

int stm32_xspi_hwenable(int intf)
{
  if (intf == 1)
    {
      modifyreg32(STM32_PWR_CSR2, 0, PWR_CSR2_EN_XSPIM1);
      modifyreg32(STM32_RCC_CCIPR1, RCC_CCIPR1_XSPI1SEL_MASK,
                  RCC_CCIPR1_XSPI1SEL_HCLK);
      modifyreg32(STM32_RCC_AHB5ENR, 0, RCC_AHB5ENR_XSPI1EN |
                   RCC_AHB5ENR_XSPIMEN);
    }
  else
    {
      modifyreg32(STM32_PWR_CSR2, 0, PWR_CSR2_EN_XSPIM2);
      modifyreg32(STM32_RCC_CCIPR1, RCC_CCIPR1_XSPI2SEL_MASK,
                  RCC_CCIPR1_XSPI2SEL_HCLK);
      modifyreg32(STM32_RCC_AHB5ENR, 0, RCC_AHB5ENR_XSPI2EN |
                   RCC_AHB5ENR_XSPIMEN);
    }

  return xspi_compensation_enable(intf);
}

#endif /* CONFIG_STM32_XSPI */
