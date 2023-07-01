/****************************************************************************
 * arch/arm/src/nrf53/nrf53_spu.c
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

#include "arm_internal.h"

#include "hardware/nrf53_spu.h"

#include "nrf53_spu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifndef CONFIG_NRF53_SPU_CUSTOM
/****************************************************************************
 * Name: nrf53_spu_configuration_default
 ****************************************************************************/

static void nrf53_spu_configuration_default(void)
{
  int i = 0;

  /* Make all flash non-secure for now */

  for (i = 0; i < 64; i++)
    {
      modifyreg32(NRF53_SPU_FLASHREGIONPERM(i),
                  SPU_FLASHREGION_PERM_SECATTR, 0);
    }

  /* Make all ram non-secure for now */

  for (i = 0; i < 64; i++)
    {
      modifyreg32(NRF53_SPU_RAMREGIONPERM(i),
                  SPU_RAMREGION_PERM_SECATTR, 0);
    }
}
#endif

/****************************************************************************
 * Name: nrf53_spu_periph
 ****************************************************************************/

static void nrf53_spu_periph(void)
{
#ifdef CONFIG_NRF91_OSCREG_NS
  modifyreg32(NRF53_SPU_PERIPHIDPERM(NRF53_OSC_REG_ID),
              SPU_PERIPHID_PERM_SECATTR, SPU_PERM_SECATTR);
#endif

#ifdef CONFIG_NRF91_POWERCLOCK_NS
  modifyreg32(NRF53_SPU_PERIPHIDPERM(NRF53_POWER_CLOCK_ID),
              SPU_PERIPHID_PERM_SECATTR, SPU_PERM_SECATTR);
#endif

#ifdef CONFIG_NRF91_GPIO01_NS
  modifyreg32(NRF53_SPU_PERIPHIDPERM(NRF53_GPIO01_ID),
              SPU_PERIPHID_PERM_SECATTR, SPU_PERM_SECATTR);
#endif

#if defined(CONFIG_NRF53_FLASH_PREFETCH) || defined(CONFIG_NRF53_PROGMEM)
  modifyreg32(NRF53_SPU_PERIPHIDPERM(NRF53_NVMC_ID),
              SPU_PERIPHID_PERM_SECATTR, SPU_PERM_SECATTR);
#endif

#ifdef CONFIG_NRF53_SERIAL0
  modifyreg32(NRF53_SPU_PERIPHIDPERM(NRF53_SERIAL0_ID),
              SPU_PERIPHID_PERM_SECATTR, SPU_PERM_SECATTR);
#endif

#ifdef CONFIG_NRF53_SERIAL1
  modifyreg32(NRF53_SPU_PERIPHIDPERM(NRF53_SERIAL1_ID),
              SPU_PERIPHID_PERM_SECATTR, SPU_PERM_SECATTR);
#endif

#ifdef CONFIG_NRF53_SERIAL2
  modifyreg32(NRF53_SPU_PERIPHIDPERM(NRF53_SERIAL2_ID),
              SPU_PERIPHID_PERM_SECATTR, SPU_PERM_SECATTR);
#endif

#ifdef CONFIG_NRF53_SERIAL3
  modifyreg32(NRF53_SPU_PERIPHIDPERM(NRF53_SERIAL3_ID),
              SPU_PERIPHID_PERM_SECATTR, SPU_PERM_SECATTR);
#endif

#if defined(CONFIG_NRF53_GPIOTE) && defined(CONFIG_ARCH_TRUSTZONE_SECURE)
  modifyreg32(NRF53_SPU_PERIPHIDPERM(NRF53_GPIOTE0_ID),
              SPU_PERIPHID_PERM_SECATTR, SPU_PERM_SECATTR);
#endif

#if defined(CONFIG_NRF53_GPIOTE) && defined(CONFIG_ARCH_TRUSTZONE_NONSECURE)
  modifyreg32(NRF53_SPU_PERIPHIDPERM(NRF53_GPIOTE1_ID),
              SPU_PERIPHID_PERM_SECATTR, SPU_PERM_SECATTR);
#endif

#ifdef CONFIG_NRF53_TIMER0
  modifyreg32(NRF53_SPU_PERIPHIDPERM(NRF53_TIMER0_ID),
              SPU_PERIPHID_PERM_SECATTR, SPU_PERM_SECATTR);
#endif

#ifdef CONFIG_NRF53_TIMER1
  modifyreg32(NRF53_SPU_PERIPHIDPERM(NRF53_TIMER1_ID),
              SPU_PERIPHID_PERM_SECATTR, SPU_PERM_SECATTR);
#endif

#ifdef CONFIG_NRF53_TIMER2
  modifyreg32(NRF53_SPU_PERIPHIDPERM(NRF53_TIMER2_ID),
              SPU_PERIPHID_PERM_SECATTR, SPU_PERM_SECATTR);
#endif

#ifdef CONFIG_NRF53_RTC0
  modifyreg32(NRF53_SPU_PERIPHIDPERM(NRF53_RTC0_ID),
              SPU_PERIPHID_PERM_SECATTR, SPU_PERM_SECATTR);
#endif

#ifdef CONFIG_NRF53_RTC1
  modifyreg32(NRF53_SPU_PERIPHIDPERM(NRF53_RTC1_ID),
              SPU_PERIPHID_PERM_SECATTR, SPU_PERM_SECATTR);
#endif

#ifdef CONFIG_NRF53_COMP
  modifyreg32(NRF53_SPU_PERIPHIDPERM(NRF53_COMP_ID),
              SPU_PERIPHID_PERM_SECATTR, SPU_PERM_SECATTR);
#endif

#ifdef CONFIG_NRF53_WDT0
  modifyreg32(NRF53_SPU_PERIPHIDPERM(NRF53_WDT0_ID),
              SPU_PERIPHID_PERM_SECATTR, SPU_PERM_SECATTR);
#endif

#ifdef CONFIG_NRF53_WDT1
  modifyreg32(NRF53_SPU_PERIPHIDPERM(NRF53_WDT1_ID),
              SPU_PERIPHID_PERM_SECATTR, SPU_PERM_SECATTR);
#endif

#ifdef CONFIG_NRF53_PWM0
  modifyreg32(NRF53_SPU_PERIPHIDPERM(NRF53_PWM0_ID),
              SPU_PERIPHID_PERM_SECATTR, SPU_PERM_SECATTR);
#endif

#ifdef CONFIG_NRF53_PWM1
  modifyreg32(NRF53_SPU_PERIPHIDPERM(NRF53_PWM1_ID),
              SPU_PERIPHID_PERM_SECATTR, SPU_PERM_SECATTR);
#endif

#ifdef CONFIG_NRF53_PWM2
  modifyreg32(NRF53_SPU_PERIPHIDPERM(NRF53_PWM2_ID),
              SPU_PERIPHID_PERM_SECATTR, SPU_PERM_SECATTR);
#endif

#ifdef CONFIG_NRF53_IPC
  modifyreg32(NRF53_SPU_PERIPHIDPERM(NRF53_IPC_ID),
              SPU_PERIPHID_PERM_SECATTR, SPU_PERM_SECATTR);
#endif

#ifdef CONFIG_NRF53_QSPI
  modifyreg32(NRF53_SPU_PERIPHIDPERM(NRF53_QSPI_ID),
              SPU_PERIPHID_PERM_SECATTR, SPU_PERM_SECATTR);
#endif

#ifdef CONFIG_NRF53_NFCT
  modifyreg32(NRF53_SPU_PERIPHIDPERM(NRF53_NFCT_ID),
              SPU_PERIPHID_PERM_SECATTR, SPU_PERM_SECATTR);
#endif

#ifdef CONFIG_NRF53_USBD
  modifyreg32(NRF53_SPU_PERIPHIDPERM(NRF53_USBD_ID),
              SPU_PERIPHID_PERM_SECATTR, SPU_PERM_SECATTR);
#endif

  /* Make all GPIO non-secure */

  modifyreg32(NRF53_SPU_GPIOPORTPERM(0), 0xffffffff, 0);
  modifyreg32(NRF53_SPU_GPIOPORTPERM(1), 0xffffffff, 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef HAVE_SPU_CONFIG
/****************************************************************************
 * Name: nrf53_spu_configure
 ****************************************************************************/

void nrf53_spu_configure(void)
{
#ifdef CONFIG_RPTUN
  /* Set secure domain - this allows net core to access shared mem */

  putreg32(SPU_EXTDOMAIN_SECUREMAPPING_SECATTR, NRF53_SPU_EXTDOMAIN(0));
#endif

#if defined(NRF91_SPU_NONSECURE)
  /* Peripheral configuration */

  nrf53_spu_periph();

  /* Memory SPU configuration */

  nrf53_spu_configuration_default();
#elif defined(CONFIG_NRF91_SPU_CUSTOM)
  /* User-specific SPU configuration */

  board_spu_configure();
#endif
}
#endif
