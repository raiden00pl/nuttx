/****************************************************************************
 * arch/arm/src/stm32n6/hardware/stm32n6xxx_pinmap.h
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

#ifndef __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_PINMAP_H
#define __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_PINMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Alternate Pin Functions.
 *
 * Alternative pin selections are provided with a numeric suffix like _1, _2,
 * etc.  Drivers, however, will use the pin selection without the numeric
 * suffix.  Additional definitions are required in the board.h file.  For
 * example, if USART1_TX connects via PE5 on some board, then the following
 * definition should appear in the board.h header file for that board:
 *
 * #define GPIO_USART1_TX GPIO_USART1_TX_1
 *
 * The driver will then automatically configure PE5 as the USART1 TX pin.
 */

/* USART1: PE5=TX (AF7), PE6=RX (AF7) - ST-Link Virtual COM Port */

/* ADC */

#define GPIO_ADC1_INP10_1  (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN9)
#define GPIO_ADC1_INP11_1  (GPIO_ANALOG | GPIO_PORTA | GPIO_PIN10)

/* ETH1: MDIO and RGMII */

#define GPIO_ETH_MDC_1             (GPIO_ALT | GPIO_AF11 | GPIO_PUSHPULL | GPIO_PORTD | GPIO_PIN1)
#define GPIO_ETH_MDIO_1            (GPIO_ALT | GPIO_AF11 | GPIO_PUSHPULL | GPIO_PORTD | GPIO_PIN12)
#define GPIO_ETH_RGMII_CLK125_1    (GPIO_ALT | GPIO_AF11 | GPIO_PORTF | GPIO_PIN2)
#define GPIO_ETH_RGMII_GTX_CLK_1   (GPIO_ALT | GPIO_AF12 | GPIO_PUSHPULL | GPIO_PORTF | GPIO_PIN0)
#define GPIO_ETH_RGMII_RX_CLK_1    (GPIO_ALT | GPIO_AF11 | GPIO_PORTF | GPIO_PIN7)
#define GPIO_ETH_RGMII_RX_CTL_1    (GPIO_ALT | GPIO_AF11 | GPIO_PORTF | GPIO_PIN10)
#define GPIO_ETH_RGMII_RXD0_1      (GPIO_ALT | GPIO_AF11 | GPIO_PORTF | GPIO_PIN14)
#define GPIO_ETH_RGMII_RXD1_1      (GPIO_ALT | GPIO_AF11 | GPIO_PORTF | GPIO_PIN15)
#define GPIO_ETH_RGMII_RXD2_1      (GPIO_ALT | GPIO_AF11 | GPIO_PORTF | GPIO_PIN8)
#define GPIO_ETH_RGMII_RXD3_1      (GPIO_ALT | GPIO_AF11 | GPIO_PORTF | GPIO_PIN9)
#define GPIO_ETH_RGMII_TX_CTL_1    (GPIO_ALT | GPIO_AF11 | GPIO_PUSHPULL | GPIO_PORTF | GPIO_PIN11)
#define GPIO_ETH_RGMII_TXD0_1      (GPIO_ALT | GPIO_AF11 | GPIO_PUSHPULL | GPIO_PORTF | GPIO_PIN12)
#define GPIO_ETH_RGMII_TXD1_1      (GPIO_ALT | GPIO_AF11 | GPIO_PUSHPULL | GPIO_PORTF | GPIO_PIN13)
#define GPIO_ETH_RGMII_TXD2_1      (GPIO_ALT | GPIO_AF11 | GPIO_PUSHPULL | GPIO_PORTG | GPIO_PIN3)
#define GPIO_ETH_RGMII_TXD3_1      (GPIO_ALT | GPIO_AF11 | GPIO_PUSHPULL | GPIO_PORTG | GPIO_PIN4)

/* I2C */

#define GPIO_I2C1_SCL_1    (GPIO_ALT | GPIO_AF4 | GPIO_OPENDRAIN | GPIO_PORTH | GPIO_PIN9)
#define GPIO_I2C1_SDA_1    (GPIO_ALT | GPIO_AF4 | GPIO_OPENDRAIN | GPIO_PORTC | GPIO_PIN1)
#define GPIO_I2C2_SCL_1    (GPIO_ALT | GPIO_AF4 | GPIO_OPENDRAIN | GPIO_PORTD | GPIO_PIN14)
#define GPIO_I2C2_SDA_1    (GPIO_ALT | GPIO_AF4 | GPIO_OPENDRAIN | GPIO_PORTD | GPIO_PIN4)
#define GPIO_I2C4_SCL_1    (GPIO_ALT | GPIO_AF4 | GPIO_OPENDRAIN | GPIO_PORTE | GPIO_PIN13)
#define GPIO_I2C4_SDA_1    (GPIO_ALT | GPIO_AF4 | GPIO_OPENDRAIN | GPIO_PORTE | GPIO_PIN14)

/* LTDC */

#define GPIO_LTDC_R0_1     (GPIO_ALT | GPIO_AF14 | GPIO_PUSHPULL | GPIO_PORTG | GPIO_PIN0)
#define GPIO_LTDC_R1_1     (GPIO_ALT | GPIO_AF14 | GPIO_PUSHPULL | GPIO_PORTD | GPIO_PIN9)
#define GPIO_LTDC_R2_1     (GPIO_ALT | GPIO_AF14 | GPIO_PUSHPULL | GPIO_PORTD | GPIO_PIN15)
#define GPIO_LTDC_R3_1     (GPIO_ALT | GPIO_AF14 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN4)
#define GPIO_LTDC_R4_1     (GPIO_ALT | GPIO_AF14 | GPIO_PUSHPULL | GPIO_PORTH | GPIO_PIN4)
#define GPIO_LTDC_R5_1     (GPIO_ALT | GPIO_AF14 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN15)
#define GPIO_LTDC_R6_1     (GPIO_ALT | GPIO_AF14 | GPIO_PUSHPULL | GPIO_PORTG | GPIO_PIN11)
#define GPIO_LTDC_R7_1     (GPIO_ALT | GPIO_AF14 | GPIO_PUSHPULL | GPIO_PORTD | GPIO_PIN8)
#define GPIO_LTDC_G0_1     (GPIO_ALT | GPIO_AF14 | GPIO_PUSHPULL | GPIO_PORTG | GPIO_PIN12)
#define GPIO_LTDC_G1_1     (GPIO_ALT | GPIO_AF14 | GPIO_PUSHPULL | GPIO_PORTG | GPIO_PIN1)
#define GPIO_LTDC_G2_1     (GPIO_ALT | GPIO_AF14 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN1)
#define GPIO_LTDC_G3_1     (GPIO_ALT | GPIO_AF14 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN0)
#define GPIO_LTDC_G4_1     (GPIO_ALT | GPIO_AF14 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN15)
#define GPIO_LTDC_G5_1     (GPIO_ALT | GPIO_AF14 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN12)
#define GPIO_LTDC_G6_1     (GPIO_ALT | GPIO_AF14 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN11)
#define GPIO_LTDC_G7_1     (GPIO_ALT | GPIO_AF14 | GPIO_PUSHPULL | GPIO_PORTG | GPIO_PIN8)
#define GPIO_LTDC_B0_1     (GPIO_ALT | GPIO_AF14 | GPIO_PUSHPULL | GPIO_PORTG | GPIO_PIN15)
#define GPIO_LTDC_B1_1     (GPIO_ALT | GPIO_AF14 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN7)
#define GPIO_LTDC_B2_1     (GPIO_ALT | GPIO_AF14 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN2)
#define GPIO_LTDC_B3_1     (GPIO_ALT | GPIO_AF14 | GPIO_PUSHPULL | GPIO_PORTG | GPIO_PIN6)
#define GPIO_LTDC_B4_1     (GPIO_ALT | GPIO_AF14 | GPIO_PUSHPULL | GPIO_PORTH | GPIO_PIN3)
#define GPIO_LTDC_B5_1     (GPIO_ALT | GPIO_AF14 | GPIO_PUSHPULL | GPIO_PORTH | GPIO_PIN6)
#define GPIO_LTDC_B6_1     (GPIO_ALT | GPIO_AF14 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN8)
#define GPIO_LTDC_B7_1     (GPIO_ALT | GPIO_AF14 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN2)
#define GPIO_LTDC_CLK_1    (GPIO_ALT | GPIO_AF14 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN13)
#define GPIO_LTDC_DE_1     (GPIO_ALT | GPIO_AF14 | GPIO_PUSHPULL | GPIO_PORTG | GPIO_PIN13)
#define GPIO_LTDC_HSYNC_1  (GPIO_ALT | GPIO_AF14 | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN14)
#define GPIO_LTDC_VSYNC_1  (GPIO_ALT | GPIO_AF14 | GPIO_PUSHPULL | GPIO_PORTE | GPIO_PIN11)

/* SDMMC */

#define GPIO_SDMMC2_CK_1   (GPIO_ALT | GPIO_AF11 | GPIO_PUSHPULL | GPIO_PORTC | GPIO_PIN2)
#define GPIO_SDMMC2_CMD_1  (GPIO_ALT | GPIO_AF11 | GPIO_PUSHPULL | GPIO_PORTC | GPIO_PIN3)
#define GPIO_SDMMC2_D0_1   (GPIO_ALT | GPIO_AF11 | GPIO_PUSHPULL | GPIO_PORTC | GPIO_PIN4)
#define GPIO_SDMMC2_D1_1   (GPIO_ALT | GPIO_AF11 | GPIO_PUSHPULL | GPIO_PORTC | GPIO_PIN5)
#define GPIO_SDMMC2_D2_1   (GPIO_ALT | GPIO_AF11 | GPIO_PUSHPULL | GPIO_PORTC | GPIO_PIN0)
#define GPIO_SDMMC2_D3_1   (GPIO_ALT | GPIO_AF11 | GPIO_PUSHPULL | GPIO_PORTE | GPIO_PIN4)

/* SPI */

#define GPIO_SPI5_SCK_1    (GPIO_ALT | GPIO_AF5 | GPIO_PUSHPULL | GPIO_PORTE | GPIO_PIN15)
#define GPIO_SPI5_MISO_1   (GPIO_ALT | GPIO_AF5 | GPIO_PORTH | GPIO_PIN8)
#define GPIO_SPI5_MOSI_1   (GPIO_ALT | GPIO_AF5 | GPIO_PUSHPULL | GPIO_PORTG | GPIO_PIN2)
#define GPIO_SPI5_NSS_1    (GPIO_ALT | GPIO_AF5 | GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN3)

/* TIM */

#define GPIO_TIM1_CH1OUT_1 (GPIO_ALT | GPIO_AF1 | GPIO_PUSHPULL | GPIO_PORTE | GPIO_PIN9)
#define GPIO_TIM15_CH1OUT_1 (GPIO_ALT | GPIO_AF4 | GPIO_PUSHPULL | GPIO_PORTC | GPIO_PIN12)

/* USART */

#define GPIO_USART1_TX_1   (GPIO_ALT | GPIO_AF7 | GPIO_SPEED_50MHZ | GPIO_PUSHPULL | GPIO_PORTE | GPIO_PIN5)
#define GPIO_USART1_RX_1   (GPIO_ALT | GPIO_AF7 | GPIO_SPEED_50MHZ | GPIO_PORTE | GPIO_PIN6)
#define GPIO_USART2_TX_1   (GPIO_ALT | GPIO_AF7 | GPIO_PUSHPULL | GPIO_PORTD | GPIO_PIN5)
#define GPIO_USART2_RX_1   (GPIO_ALT | GPIO_AF7 | GPIO_PORTF | GPIO_PIN6)
#define GPIO_USART2_CTS_1  (GPIO_ALT | GPIO_AF7 | GPIO_PORTG | GPIO_PIN5)
#define GPIO_USART2_RTS_1  (GPIO_ALT | GPIO_AF8 | GPIO_PUSHPULL | GPIO_PORTG | GPIO_PIN14)

#endif /* __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_PINMAP_H */
