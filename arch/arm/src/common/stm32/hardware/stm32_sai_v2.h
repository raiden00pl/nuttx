/****************************************************************************
 * arch/arm/src/common/stm32/hardware/stm32_sai_v2.h
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

#ifndef __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_SAI_V2_H
#define __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_SAI_V2_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* SAI v2 (STM32N6) keeps the v1 register layout and only extends
 * the configuration register 1.
 */

#include "hardware/stm32_sai_v1.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SAI Configuration Register 1: the master clock divider is 6 bits wide
 * (bits 20-25), bits 26-27 are added and bits 28-31 are reserved.
 */

#undef  SAI_CR1_MCKDIV_MASK
#define SAI_CR1_MCKDIV_MASK        (63 << SAI_CR1_MCKDIV_SHIFT)
#define SAI_CR1_OSR                (1 << 26) /* Bit 26: Oversampling ratio for master clock */
#define SAI_CR1_MCKEN              (1 << 27) /* Bit 27: Master clock generation enable */

#endif /* __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_SAI_V2_H */
