/****************************************************************************
 * boards/arm/common/stm32/include/stm32_eeprma1.h
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

#ifndef __BOARDS_ARM_COMMON_STM32_INCLUDE_STM32_EEPRMA1_H
#define __BOARDS_ARM_COMMON_STM32_INCLUDE_STM32_EEPRMA1_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: board_eeprma1_initialize
 *
 * Description:
 *   Register the M95M01, M95256 and M95040 of the X-NUCLEO-EEPRMA1 shield
 *   as /dev/eeprom0, /dev/eeprom1 and /dev/eeprom2.
 *
 * Input Parameters:
 *   busno - SPI bus of the Arduino connector
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_eeprma1_initialize(int busno);

/****************************************************************************
 * Name: board_eeprma1_select
 *
 * Description:
 *   Drive the chip select of a shield EEPROM.  Called from the board
 *   stm32_spiNselect() of the Arduino SPI bus.
 *
 * Input Parameters:
 *   devid    - SPIDEV_EEPROM(0..2)
 *   selected - true to assert the chip select
 *
 ****************************************************************************/

void board_eeprma1_select(uint32_t devid, bool selected);

#ifdef __cplusplus
}
#endif

#endif /* __BOARDS_ARM_COMMON_STM32_INCLUDE_STM32_EEPRMA1_H */
