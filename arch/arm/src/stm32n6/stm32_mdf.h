/****************************************************************************
 * arch/arm/src/stm32n6/stm32_mdf.h
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

#ifndef __ARCH_ARM_SRC_STM32N6_STM32_MDF_H
#define __ARCH_ARM_SRC_STM32N6_STM32_MDF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/audio/audio.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_mdf_initialize
 *
 * Description:
 *   Initialize MDF1 filter 0 for a PDM microphone clocked by MDF1_CCK0
 *   with its data on MDF1_SDI0 and return an audio capture lower half.
 *   The board provides GPIO_MDF1_CCK0, GPIO_MDF1_SDI0 and the kernel clock
 *   STM32_MDF1_FREQUENCY.  Register the returned instance with
 *   audio_register().
 *
 * Returned Value:
 *   Audio lower half on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct audio_lowerhalf_s *stm32_mdf_initialize(void);

#endif /* __ARCH_ARM_SRC_STM32N6_STM32_MDF_H */
