/****************************************************************************
 * arch/arm/src/common/stm32/hardware/stm32_rtcc_v3.h
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

#ifndef __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_RTCC_V3_H
#define __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_RTCC_V3_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RTC v3 (STM32N6): the init/status flags live in ICSR, the event flags
 * in SR/MISR and are cleared through SCR, and the backup registers belong
 * to the TAMP peripheral.
 */

/* Register Offsets *********************************************************/

#define STM32_RTC_TR_OFFSET             0x0000
#define STM32_RTC_DR_OFFSET             0x0004
#define STM32_RTC_SSR_OFFSET            0x0008
#define STM32_RTC_ICSR_OFFSET           0x000c
#define STM32_RTC_PRER_OFFSET           0x0010
#define STM32_RTC_WUTR_OFFSET           0x0014
#define STM32_RTC_CR_OFFSET             0x0018
#define STM32_RTC_PRIVCFGR_OFFSET       0x001c
#define STM32_RTC_SECCFGR_OFFSET        0x0020
#define STM32_RTC_WPR_OFFSET            0x0024
#define STM32_RTC_CALR_OFFSET           0x0028
#define STM32_RTC_SHIFTR_OFFSET         0x002c
#define STM32_RTC_TSTR_OFFSET           0x0030
#define STM32_RTC_TSDR_OFFSET           0x0034
#define STM32_RTC_TSSSR_OFFSET          0x0038
#define STM32_RTC_ALRMAR_OFFSET         0x0040
#define STM32_RTC_ALRMASSR_OFFSET       0x0044
#define STM32_RTC_ALRMBR_OFFSET         0x0048
#define STM32_RTC_ALRMBSSR_OFFSET       0x004c
#define STM32_RTC_SR_OFFSET             0x0050
#define STM32_RTC_MISR_OFFSET           0x0054
#define STM32_RTC_SMISR_OFFSET          0x0058
#define STM32_RTC_SCR_OFFSET            0x005c
#define STM32_RTC_ALRABINR_OFFSET       0x0070
#define STM32_RTC_ALRBBINR_OFFSET       0x0074

#define STM32_TAMP_CR1_OFFSET           0x0000
#define STM32_TAMP_CR2_OFFSET           0x0004
#define STM32_TAMP_CR3_OFFSET           0x0008
#define STM32_TAMP_FLTCR_OFFSET         0x000c
#define STM32_TAMP_ATCR1_OFFSET         0x0010
#define STM32_TAMP_ATSEEDR_OFFSET       0x0014
#define STM32_TAMP_ATOR_OFFSET          0x0018
#define STM32_TAMP_ATCR2_OFFSET         0x001c
#define STM32_TAMP_SECCFGR_OFFSET       0x0020
#define STM32_TAMP_PRIVCFGR_OFFSET      0x0024
#define STM32_TAMP_IER_OFFSET           0x002c
#define STM32_TAMP_SR_OFFSET            0x0030
#define STM32_TAMP_MISR_OFFSET          0x0034
#define STM32_TAMP_SMISR_OFFSET         0x0038
#define STM32_TAMP_SCR_OFFSET           0x003c
#define STM32_TAMP_COUNT1R_OFFSET       0x0040
#define STM32_TAMP_OR_OFFSET            0x0050
#define STM32_TAMP_RPCFGR_OFFSET        0x0054
#define STM32_TAMP_BKP0R_OFFSET         0x0100
#define STM32_TAMP_BKP1R_OFFSET         0x0104
#define STM32_TAMP_BKP2R_OFFSET         0x0108
#define STM32_TAMP_BKP3R_OFFSET         0x010c
#define STM32_TAMP_BKP4R_OFFSET         0x0110
#define STM32_TAMP_BKP5R_OFFSET         0x0114
#define STM32_TAMP_BKP6R_OFFSET         0x0118
#define STM32_TAMP_BKP7R_OFFSET         0x011c
#define STM32_TAMP_BKP8R_OFFSET         0x0120
#define STM32_TAMP_BKP9R_OFFSET         0x0124
#define STM32_TAMP_BKP10R_OFFSET        0x0128
#define STM32_TAMP_BKP11R_OFFSET        0x012c
#define STM32_TAMP_BKP12R_OFFSET        0x0130
#define STM32_TAMP_BKP13R_OFFSET        0x0134
#define STM32_TAMP_BKP14R_OFFSET        0x0138
#define STM32_TAMP_BKP15R_OFFSET        0x013c
#define STM32_TAMP_BKP16R_OFFSET        0x0140
#define STM32_TAMP_BKP17R_OFFSET        0x0144
#define STM32_TAMP_BKP18R_OFFSET        0x0148
#define STM32_TAMP_BKP19R_OFFSET        0x014c
#define STM32_TAMP_BKP20R_OFFSET        0x0150
#define STM32_TAMP_BKP21R_OFFSET        0x0154
#define STM32_TAMP_BKP22R_OFFSET        0x0158
#define STM32_TAMP_BKP23R_OFFSET        0x015c
#define STM32_TAMP_BKP24R_OFFSET        0x0160
#define STM32_TAMP_BKP25R_OFFSET        0x0164
#define STM32_TAMP_BKP26R_OFFSET        0x0168
#define STM32_TAMP_BKP27R_OFFSET        0x016c
#define STM32_TAMP_BKP28R_OFFSET        0x0170
#define STM32_TAMP_BKP29R_OFFSET        0x0174
#define STM32_TAMP_BKP30R_OFFSET        0x0178
#define STM32_TAMP_BKP31R_OFFSET        0x017c

#define STM32_TAMP_BKPR_OFFSET(n)       (0x0100 + ((n) << 2))

/* Register Addresses *******************************************************/

#define STM32_RTC_TR                    (STM32_RTC_BASE + STM32_RTC_TR_OFFSET)
#define STM32_RTC_DR                    (STM32_RTC_BASE + STM32_RTC_DR_OFFSET)
#define STM32_RTC_SSR                   (STM32_RTC_BASE + STM32_RTC_SSR_OFFSET)
#define STM32_RTC_ICSR                  (STM32_RTC_BASE + STM32_RTC_ICSR_OFFSET)
#define STM32_RTC_PRER                  (STM32_RTC_BASE + STM32_RTC_PRER_OFFSET)
#define STM32_RTC_WUTR                  (STM32_RTC_BASE + STM32_RTC_WUTR_OFFSET)
#define STM32_RTC_CR                    (STM32_RTC_BASE + STM32_RTC_CR_OFFSET)
#define STM32_RTC_PRIVCFGR              (STM32_RTC_BASE + STM32_RTC_PRIVCFGR_OFFSET)
#define STM32_RTC_SECCFGR               (STM32_RTC_BASE + STM32_RTC_SECCFGR_OFFSET)
#define STM32_RTC_WPR                   (STM32_RTC_BASE + STM32_RTC_WPR_OFFSET)
#define STM32_RTC_CALR                  (STM32_RTC_BASE + STM32_RTC_CALR_OFFSET)
#define STM32_RTC_SHIFTR                (STM32_RTC_BASE + STM32_RTC_SHIFTR_OFFSET)
#define STM32_RTC_TSTR                  (STM32_RTC_BASE + STM32_RTC_TSTR_OFFSET)
#define STM32_RTC_TSDR                  (STM32_RTC_BASE + STM32_RTC_TSDR_OFFSET)
#define STM32_RTC_TSSSR                 (STM32_RTC_BASE + STM32_RTC_TSSSR_OFFSET)
#define STM32_RTC_ALRMAR                (STM32_RTC_BASE + STM32_RTC_ALRMAR_OFFSET)
#define STM32_RTC_ALRMASSR              (STM32_RTC_BASE + STM32_RTC_ALRMASSR_OFFSET)
#define STM32_RTC_ALRMBR                (STM32_RTC_BASE + STM32_RTC_ALRMBR_OFFSET)
#define STM32_RTC_ALRMBSSR              (STM32_RTC_BASE + STM32_RTC_ALRMBSSR_OFFSET)
#define STM32_RTC_SR                    (STM32_RTC_BASE + STM32_RTC_SR_OFFSET)
#define STM32_RTC_MISR                  (STM32_RTC_BASE + STM32_RTC_MISR_OFFSET)
#define STM32_RTC_SMISR                 (STM32_RTC_BASE + STM32_RTC_SMISR_OFFSET)
#define STM32_RTC_SCR                   (STM32_RTC_BASE + STM32_RTC_SCR_OFFSET)
#define STM32_RTC_ALRABINR              (STM32_RTC_BASE + STM32_RTC_ALRABINR_OFFSET)
#define STM32_RTC_ALRBBINR              (STM32_RTC_BASE + STM32_RTC_ALRBBINR_OFFSET)

#define STM32_TAMP_CR1                  (STM32_TAMP_BASE + STM32_TAMP_CR1_OFFSET)
#define STM32_TAMP_CR2                  (STM32_TAMP_BASE + STM32_TAMP_CR2_OFFSET)
#define STM32_TAMP_CR3                  (STM32_TAMP_BASE + STM32_TAMP_CR3_OFFSET)
#define STM32_TAMP_FLTCR                (STM32_TAMP_BASE + STM32_TAMP_FLTCR_OFFSET)
#define STM32_TAMP_ATCR1                (STM32_TAMP_BASE + STM32_TAMP_ATCR1_OFFSET)
#define STM32_TAMP_ATSEEDR              (STM32_TAMP_BASE + STM32_TAMP_ATSEEDR_OFFSET)
#define STM32_TAMP_ATOR                 (STM32_TAMP_BASE + STM32_TAMP_ATOR_OFFSET)
#define STM32_TAMP_ATCR2                (STM32_TAMP_BASE + STM32_TAMP_ATCR2_OFFSET)
#define STM32_TAMP_SECCFGR              (STM32_TAMP_BASE + STM32_TAMP_SECCFGR_OFFSET)
#define STM32_TAMP_PRIVCFGR             (STM32_TAMP_BASE + STM32_TAMP_PRIVCFGR_OFFSET)
#define STM32_TAMP_IER                  (STM32_TAMP_BASE + STM32_TAMP_IER_OFFSET)
#define STM32_TAMP_SR                   (STM32_TAMP_BASE + STM32_TAMP_SR_OFFSET)
#define STM32_TAMP_MISR                 (STM32_TAMP_BASE + STM32_TAMP_MISR_OFFSET)
#define STM32_TAMP_SMISR                (STM32_TAMP_BASE + STM32_TAMP_SMISR_OFFSET)
#define STM32_TAMP_SCR                  (STM32_TAMP_BASE + STM32_TAMP_SCR_OFFSET)
#define STM32_TAMP_COUNT1R              (STM32_TAMP_BASE + STM32_TAMP_COUNT1R_OFFSET)
#define STM32_TAMP_OR                   (STM32_TAMP_BASE + STM32_TAMP_OR_OFFSET)
#define STM32_TAMP_RPCFGR               (STM32_TAMP_BASE + STM32_TAMP_RPCFGR_OFFSET)

#define STM32_TAMP_BKPR(n)              (STM32_TAMP_BASE + STM32_TAMP_BKPR_OFFSET(n))

/* Register Bitfield Definitions ********************************************/

/* RTC TR register */

#define RTC_TR_SU_SHIFT                 (0)                   /* Bits 0-3 */
#define RTC_TR_SU_MASK                  (0xf << RTC_TR_SU_SHIFT)
#define RTC_TR_ST_SHIFT                 (4)                   /* Bits 4-6 */
#define RTC_TR_ST_MASK                  (0x7 << RTC_TR_ST_SHIFT)
#define RTC_TR_MNU_SHIFT                (8)                   /* Bits 8-11 */
#define RTC_TR_MNU_MASK                 (0xf << RTC_TR_MNU_SHIFT)
#define RTC_TR_MNT_SHIFT                (12)                  /* Bits 12-14 */
#define RTC_TR_MNT_MASK                 (0x7 << RTC_TR_MNT_SHIFT)
#define RTC_TR_HU_SHIFT                 (16)                  /* Bits 16-19 */
#define RTC_TR_HU_MASK                  (0xf << RTC_TR_HU_SHIFT)
#define RTC_TR_HT_SHIFT                 (20)                  /* Bits 20-21 */
#define RTC_TR_HT_MASK                  (0x3 << RTC_TR_HT_SHIFT)
#define RTC_TR_PM                       (1 << 22)             /* Bit 22 */

/* RTC DR register */

#define RTC_DR_DU_SHIFT                 (0)                   /* Bits 0-3 */
#define RTC_DR_DU_MASK                  (0xf << RTC_DR_DU_SHIFT)
#define RTC_DR_DT_SHIFT                 (4)                   /* Bits 4-5 */
#define RTC_DR_DT_MASK                  (0x3 << RTC_DR_DT_SHIFT)
#define RTC_DR_MU_SHIFT                 (8)                   /* Bits 8-11 */
#define RTC_DR_MU_MASK                  (0xf << RTC_DR_MU_SHIFT)
#define RTC_DR_MT                       (1 << 12)             /* Bit 12 */
#define RTC_DR_WDU_SHIFT                (13)                  /* Bits 13-15 */
#define RTC_DR_WDU_MASK                 (0x7 << RTC_DR_WDU_SHIFT)
#define RTC_DR_YU_SHIFT                 (16)                  /* Bits 16-19 */
#define RTC_DR_YU_MASK                  (0xf << RTC_DR_YU_SHIFT)
#define RTC_DR_YT_SHIFT                 (20)                  /* Bits 20-23 */
#define RTC_DR_YT_MASK                  (0xf << RTC_DR_YT_SHIFT)

/* RTC SSR register */

#define RTC_SSR_SS_SHIFT                (0)                   /* Bits 0-31 */
#define RTC_SSR_SS_MASK                 (0xffffffff << RTC_SSR_SS_SHIFT)

/* RTC ICSR register */

#define RTC_ICSR_WUTWF                  (1 << 2)              /* Bit 2 */
#define RTC_ICSR_SHPF                   (1 << 3)              /* Bit 3 */
#define RTC_ICSR_INITS                  (1 << 4)              /* Bit 4 */
#define RTC_ICSR_RSF                    (1 << 5)              /* Bit 5 */
#define RTC_ICSR_INITF                  (1 << 6)              /* Bit 6 */
#define RTC_ICSR_INIT                   (1 << 7)              /* Bit 7 */
#define RTC_ICSR_BIN_SHIFT              (8)                   /* Bits 8-9 */
#define RTC_ICSR_BIN_MASK               (0x3 << RTC_ICSR_BIN_SHIFT)
#define RTC_ICSR_BCDU_SHIFT             (10)                  /* Bits 10-12 */
#define RTC_ICSR_BCDU_MASK              (0x7 << RTC_ICSR_BCDU_SHIFT)
#define RTC_ICSR_RECALPF                (1 << 16)             /* Bit 16 */

/* RTC PRER register */

#define RTC_PRER_PREDIV_S_SHIFT         (0)                   /* Bits 0-14 */
#define RTC_PRER_PREDIV_S_MASK          (0x7fff << RTC_PRER_PREDIV_S_SHIFT)
#define RTC_PRER_PREDIV_A_SHIFT         (16)                  /* Bits 16-22 */
#define RTC_PRER_PREDIV_A_MASK          (0x7f << RTC_PRER_PREDIV_A_SHIFT)

/* RTC WUTR register */

#define RTC_WUTR_WUT_SHIFT              (0)                   /* Bits 0-15 */
#define RTC_WUTR_WUT_MASK               (0xffff << RTC_WUTR_WUT_SHIFT)
#define RTC_WUTR_WUTOCLR_SHIFT          (16)                  /* Bits 16-31 */
#define RTC_WUTR_WUTOCLR_MASK           (0xffff << RTC_WUTR_WUTOCLR_SHIFT)

/* RTC CR register */

#define RTC_CR_WUCKSEL_SHIFT            (0)                   /* Bits 0-2 */
#define RTC_CR_WUCKSEL_MASK             (0x7 << RTC_CR_WUCKSEL_SHIFT)
#define RTC_CR_WUCKSEL_RTCDIV16         (0 << RTC_CR_WUCKSEL_SHIFT)
#define RTC_CR_WUCKSEL_RTCDIV8          (1 << RTC_CR_WUCKSEL_SHIFT)
#define RTC_CR_WUCKSEL_RTCDIV4          (2 << RTC_CR_WUCKSEL_SHIFT)
#define RTC_CR_WUCKSEL_RTCDIV2          (3 << RTC_CR_WUCKSEL_SHIFT)
#define RTC_CR_WUCKSEL_CKSPRE           (4 << RTC_CR_WUCKSEL_SHIFT)
#define RTC_CR_WUCKSEL_CKSPRE_ADD1      (6 << RTC_CR_WUCKSEL_SHIFT)
#define RTC_CR_TSEDGE                   (1 << 3)              /* Bit 3 */
#define RTC_CR_REFCKON                  (1 << 4)              /* Bit 4 */
#define RTC_CR_BYPSHAD                  (1 << 5)              /* Bit 5 */
#define RTC_CR_FMT                      (1 << 6)              /* Bit 6 */
#define RTC_CR_SSRUIE                   (1 << 7)              /* Bit 7 */
#define RTC_CR_ALRAE                    (1 << 8)              /* Bit 8 */
#define RTC_CR_ALRBE                    (1 << 9)              /* Bit 9 */
#define RTC_CR_WUTE                     (1 << 10)             /* Bit 10 */
#define RTC_CR_TSE                      (1 << 11)             /* Bit 11 */
#define RTC_CR_ALRAIE                   (1 << 12)             /* Bit 12 */
#define RTC_CR_ALRBIE                   (1 << 13)             /* Bit 13 */
#define RTC_CR_WUTIE                    (1 << 14)             /* Bit 14 */
#define RTC_CR_TSIE                     (1 << 15)             /* Bit 15 */
#define RTC_CR_ADD1H                    (1 << 16)             /* Bit 16 */
#define RTC_CR_SUB1H                    (1 << 17)             /* Bit 17 */
#define RTC_CR_BKP                      (1 << 18)             /* Bit 18 */
#define RTC_CR_COSEL                    (1 << 19)             /* Bit 19 */
#define RTC_CR_POL                      (1 << 20)             /* Bit 20 */
#define RTC_CR_OSEL_SHIFT               (21)                  /* Bits 21-22 */
#define RTC_CR_OSEL_MASK                (0x3 << RTC_CR_OSEL_SHIFT)
#define RTC_CR_OSEL_DISABLED            (0 << RTC_CR_OSEL_SHIFT)
#define RTC_CR_OSEL_ALRMA               (1 << RTC_CR_OSEL_SHIFT)
#define RTC_CR_OSEL_ALRMB               (2 << RTC_CR_OSEL_SHIFT)
#define RTC_CR_OSEL_WAKEUP              (3 << RTC_CR_OSEL_SHIFT)
#define RTC_CR_COE                      (1 << 23)             /* Bit 23 */
#define RTC_CR_ITSE                     (1 << 24)             /* Bit 24 */
#define RTC_CR_TAMPTS                   (1 << 25)             /* Bit 25 */
#define RTC_CR_TAMPOE                   (1 << 26)             /* Bit 26 */
#define RTC_CR_ALRAFCLR                 (1 << 27)             /* Bit 27 */
#define RTC_CR_ALRBFCLR                 (1 << 28)             /* Bit 28 */
#define RTC_CR_TAMPALRM_PU              (1 << 29)             /* Bit 29 */
#define RTC_CR_TAMPALRM_TYPE            (1 << 30)             /* Bit 30 */
#define RTC_CR_OUT2EN                   (1 << 31)             /* Bit 31 */

/* RTC PRIVCFGR register */

#define RTC_PRIVCFGR_ALRAPRIV           (1 << 0)              /* Bit 0 */
#define RTC_PRIVCFGR_ALRBPRIV           (1 << 1)              /* Bit 1 */
#define RTC_PRIVCFGR_WUTPRIV            (1 << 2)              /* Bit 2 */
#define RTC_PRIVCFGR_TSPRIV             (1 << 3)              /* Bit 3 */
#define RTC_PRIVCFGR_CALPRIV            (1 << 13)             /* Bit 13 */
#define RTC_PRIVCFGR_INITPRIV           (1 << 14)             /* Bit 14 */
#define RTC_PRIVCFGR_PRIV               (1 << 15)             /* Bit 15 */

/* RTC SECCFGR register */

#define RTC_SECCFGR_ALRASEC             (1 << 0)              /* Bit 0 */
#define RTC_SECCFGR_ALRBSEC             (1 << 1)              /* Bit 1 */
#define RTC_SECCFGR_WUTSEC              (1 << 2)              /* Bit 2 */
#define RTC_SECCFGR_TSSEC               (1 << 3)              /* Bit 3 */
#define RTC_SECCFGR_CALSEC              (1 << 13)             /* Bit 13 */
#define RTC_SECCFGR_INITSEC             (1 << 14)             /* Bit 14 */
#define RTC_SECCFGR_SEC                 (1 << 15)             /* Bit 15 */

/* RTC WPR register */

#define RTC_WPR_KEY_SHIFT               (0)                   /* Bits 0-7 */
#define RTC_WPR_KEY_MASK                (0xff << RTC_WPR_KEY_SHIFT)

/* RTC CALR register */

#define RTC_CALR_CALM_SHIFT             (0)                   /* Bits 0-8 */
#define RTC_CALR_CALM_MASK              (0x1ff << RTC_CALR_CALM_SHIFT)
#define RTC_CALR_LPCAL                  (1 << 12)             /* Bit 12 */
#define RTC_CALR_CALW16                 (1 << 13)             /* Bit 13 */
#define RTC_CALR_CALW8                  (1 << 14)             /* Bit 14 */
#define RTC_CALR_CALP                   (1 << 15)             /* Bit 15 */

/* RTC SHIFTR register */

#define RTC_SHIFTR_SUBFS_SHIFT          (0)                   /* Bits 0-14 */
#define RTC_SHIFTR_SUBFS_MASK           (0x7fff << RTC_SHIFTR_SUBFS_SHIFT)
#define RTC_SHIFTR_ADD1S                (1 << 31)             /* Bit 31 */

/* RTC TSTR register */

#define RTC_TSTR_SU_SHIFT               (0)                   /* Bits 0-3 */
#define RTC_TSTR_SU_MASK                (0xf << RTC_TSTR_SU_SHIFT)
#define RTC_TSTR_ST_SHIFT               (4)                   /* Bits 4-6 */
#define RTC_TSTR_ST_MASK                (0x7 << RTC_TSTR_ST_SHIFT)
#define RTC_TSTR_MNU_SHIFT              (8)                   /* Bits 8-11 */
#define RTC_TSTR_MNU_MASK               (0xf << RTC_TSTR_MNU_SHIFT)
#define RTC_TSTR_MNT_SHIFT              (12)                  /* Bits 12-14 */
#define RTC_TSTR_MNT_MASK               (0x7 << RTC_TSTR_MNT_SHIFT)
#define RTC_TSTR_HU_SHIFT               (16)                  /* Bits 16-19 */
#define RTC_TSTR_HU_MASK                (0xf << RTC_TSTR_HU_SHIFT)
#define RTC_TSTR_HT_SHIFT               (20)                  /* Bits 20-21 */
#define RTC_TSTR_HT_MASK                (0x3 << RTC_TSTR_HT_SHIFT)
#define RTC_TSTR_PM                     (1 << 22)             /* Bit 22 */

/* RTC TSDR register */

#define RTC_TSDR_DU_SHIFT               (0)                   /* Bits 0-3 */
#define RTC_TSDR_DU_MASK                (0xf << RTC_TSDR_DU_SHIFT)
#define RTC_TSDR_DT_SHIFT               (4)                   /* Bits 4-5 */
#define RTC_TSDR_DT_MASK                (0x3 << RTC_TSDR_DT_SHIFT)
#define RTC_TSDR_MU_SHIFT               (8)                   /* Bits 8-11 */
#define RTC_TSDR_MU_MASK                (0xf << RTC_TSDR_MU_SHIFT)
#define RTC_TSDR_MT                     (1 << 12)             /* Bit 12 */
#define RTC_TSDR_WDU_SHIFT              (13)                  /* Bits 13-15 */
#define RTC_TSDR_WDU_MASK               (0x7 << RTC_TSDR_WDU_SHIFT)

/* RTC TSSSR register */

#define RTC_TSSSR_SS_SHIFT              (0)                   /* Bits 0-31 */
#define RTC_TSSSR_SS_MASK               (0xffffffff << RTC_TSSSR_SS_SHIFT)

/* RTC ALRMAR register */

#define RTC_ALRMAR_SU_SHIFT             (0)                   /* Bits 0-3 */
#define RTC_ALRMAR_SU_MASK              (0xf << RTC_ALRMAR_SU_SHIFT)
#define RTC_ALRMAR_ST_SHIFT             (4)                   /* Bits 4-6 */
#define RTC_ALRMAR_ST_MASK              (0x7 << RTC_ALRMAR_ST_SHIFT)
#define RTC_ALRMAR_MSK1                 (1 << 7)              /* Bit 7 */
#define RTC_ALRMAR_MNU_SHIFT            (8)                   /* Bits 8-11 */
#define RTC_ALRMAR_MNU_MASK             (0xf << RTC_ALRMAR_MNU_SHIFT)
#define RTC_ALRMAR_MNT_SHIFT            (12)                  /* Bits 12-14 */
#define RTC_ALRMAR_MNT_MASK             (0x7 << RTC_ALRMAR_MNT_SHIFT)
#define RTC_ALRMAR_MSK2                 (1 << 15)             /* Bit 15 */
#define RTC_ALRMAR_HU_SHIFT             (16)                  /* Bits 16-19 */
#define RTC_ALRMAR_HU_MASK              (0xf << RTC_ALRMAR_HU_SHIFT)
#define RTC_ALRMAR_HT_SHIFT             (20)                  /* Bits 20-21 */
#define RTC_ALRMAR_HT_MASK              (0x3 << RTC_ALRMAR_HT_SHIFT)
#define RTC_ALRMAR_PM                   (1 << 22)             /* Bit 22 */
#define RTC_ALRMAR_MSK3                 (1 << 23)             /* Bit 23 */
#define RTC_ALRMAR_DU_SHIFT             (24)                  /* Bits 24-27 */
#define RTC_ALRMAR_DU_MASK              (0xf << RTC_ALRMAR_DU_SHIFT)
#define RTC_ALRMAR_DT_SHIFT             (28)                  /* Bits 28-29 */
#define RTC_ALRMAR_DT_MASK              (0x3 << RTC_ALRMAR_DT_SHIFT)
#define RTC_ALRMAR_WDSEL                (1 << 30)             /* Bit 30 */
#define RTC_ALRMAR_MSK4                 (1 << 31)             /* Bit 31 */

/* RTC ALRMASSR register */

#define RTC_ALRMASSR_SS_SHIFT           (0)                   /* Bits 0-14 */
#define RTC_ALRMASSR_SS_MASK            (0x7fff << RTC_ALRMASSR_SS_SHIFT)
#define RTC_ALRMASSR_MASKSS_SHIFT       (24)                  /* Bits 24-29 */
#define RTC_ALRMASSR_MASKSS_MASK        (0x3f << RTC_ALRMASSR_MASKSS_SHIFT)
#define RTC_ALRMASSR_SSCLR              (1 << 31)             /* Bit 31 */

/* RTC ALRMBR register */

#define RTC_ALRMBR_SU_SHIFT             (0)                   /* Bits 0-3 */
#define RTC_ALRMBR_SU_MASK              (0xf << RTC_ALRMBR_SU_SHIFT)
#define RTC_ALRMBR_ST_SHIFT             (4)                   /* Bits 4-6 */
#define RTC_ALRMBR_ST_MASK              (0x7 << RTC_ALRMBR_ST_SHIFT)
#define RTC_ALRMBR_MSK1                 (1 << 7)              /* Bit 7 */
#define RTC_ALRMBR_MNU_SHIFT            (8)                   /* Bits 8-11 */
#define RTC_ALRMBR_MNU_MASK             (0xf << RTC_ALRMBR_MNU_SHIFT)
#define RTC_ALRMBR_MNT_SHIFT            (12)                  /* Bits 12-14 */
#define RTC_ALRMBR_MNT_MASK             (0x7 << RTC_ALRMBR_MNT_SHIFT)
#define RTC_ALRMBR_MSK2                 (1 << 15)             /* Bit 15 */
#define RTC_ALRMBR_HU_SHIFT             (16)                  /* Bits 16-19 */
#define RTC_ALRMBR_HU_MASK              (0xf << RTC_ALRMBR_HU_SHIFT)
#define RTC_ALRMBR_HT_SHIFT             (20)                  /* Bits 20-21 */
#define RTC_ALRMBR_HT_MASK              (0x3 << RTC_ALRMBR_HT_SHIFT)
#define RTC_ALRMBR_PM                   (1 << 22)             /* Bit 22 */
#define RTC_ALRMBR_MSK3                 (1 << 23)             /* Bit 23 */
#define RTC_ALRMBR_DU_SHIFT             (24)                  /* Bits 24-27 */
#define RTC_ALRMBR_DU_MASK              (0xf << RTC_ALRMBR_DU_SHIFT)
#define RTC_ALRMBR_DT_SHIFT             (28)                  /* Bits 28-29 */
#define RTC_ALRMBR_DT_MASK              (0x3 << RTC_ALRMBR_DT_SHIFT)
#define RTC_ALRMBR_WDSEL                (1 << 30)             /* Bit 30 */
#define RTC_ALRMBR_MSK4                 (1 << 31)             /* Bit 31 */

/* RTC ALRMBSSR register */

#define RTC_ALRMBSSR_SS_SHIFT           (0)                   /* Bits 0-14 */
#define RTC_ALRMBSSR_SS_MASK            (0x7fff << RTC_ALRMBSSR_SS_SHIFT)
#define RTC_ALRMBSSR_MASKSS_SHIFT       (24)                  /* Bits 24-29 */
#define RTC_ALRMBSSR_MASKSS_MASK        (0x3f << RTC_ALRMBSSR_MASKSS_SHIFT)
#define RTC_ALRMBSSR_SSCLR              (1 << 31)             /* Bit 31 */

/* RTC SR register */

#define RTC_SR_ALRAF                    (1 << 0)              /* Bit 0 */
#define RTC_SR_ALRBF                    (1 << 1)              /* Bit 1 */
#define RTC_SR_WUTF                     (1 << 2)              /* Bit 2 */
#define RTC_SR_TSF                      (1 << 3)              /* Bit 3 */
#define RTC_SR_TSOVF                    (1 << 4)              /* Bit 4 */
#define RTC_SR_ITSF                     (1 << 5)              /* Bit 5 */
#define RTC_SR_SSRUF                    (1 << 6)              /* Bit 6 */

/* RTC MISR register */

#define RTC_MISR_ALRAMF                 (1 << 0)              /* Bit 0 */
#define RTC_MISR_ALRBMF                 (1 << 1)              /* Bit 1 */
#define RTC_MISR_WUTMF                  (1 << 2)              /* Bit 2 */
#define RTC_MISR_TSMF                   (1 << 3)              /* Bit 3 */
#define RTC_MISR_TSOVMF                 (1 << 4)              /* Bit 4 */
#define RTC_MISR_ITSMF                  (1 << 5)              /* Bit 5 */
#define RTC_MISR_SSRUMF                 (1 << 6)              /* Bit 6 */

/* RTC SMISR register */

#define RTC_SMISR_ALRAMF                (1 << 0)              /* Bit 0 */
#define RTC_SMISR_ALRBMF                (1 << 1)              /* Bit 1 */
#define RTC_SMISR_WUTMF                 (1 << 2)              /* Bit 2 */
#define RTC_SMISR_TSMF                  (1 << 3)              /* Bit 3 */
#define RTC_SMISR_TSOVMF                (1 << 4)              /* Bit 4 */
#define RTC_SMISR_ITSMF                 (1 << 5)              /* Bit 5 */
#define RTC_SMISR_SSRUMF                (1 << 6)              /* Bit 6 */

/* RTC SCR register */

#define RTC_SCR_CALRAF                  (1 << 0)              /* Bit 0 */
#define RTC_SCR_CALRBF                  (1 << 1)              /* Bit 1 */
#define RTC_SCR_CWUTF                   (1 << 2)              /* Bit 2 */
#define RTC_SCR_CTSF                    (1 << 3)              /* Bit 3 */
#define RTC_SCR_CTSOVF                  (1 << 4)              /* Bit 4 */
#define RTC_SCR_CITSF                   (1 << 5)              /* Bit 5 */
#define RTC_SCR_CSSRUF                  (1 << 6)              /* Bit 6 */

/* RTC ALRABINR register */

#define RTC_ALRABINR_SS_SHIFT           (0)                   /* Bits 0-31 */
#define RTC_ALRABINR_SS_MASK            (0xffffffff << RTC_ALRABINR_SS_SHIFT)

/* RTC ALRBBINR register */

#define RTC_ALRBBINR_SS_SHIFT           (0)                   /* Bits 0-31 */
#define RTC_ALRBBINR_SS_MASK            (0xffffffff << RTC_ALRBBINR_SS_SHIFT)

/* TAMP CR1 register */

#define TAMP_CR1_TAMP1E                 (1 << 0)              /* Bit 0 */
#define TAMP_CR1_TAMP2E                 (1 << 1)              /* Bit 1 */
#define TAMP_CR1_TAMP3E                 (1 << 2)              /* Bit 2 */
#define TAMP_CR1_TAMP4E                 (1 << 3)              /* Bit 3 */
#define TAMP_CR1_TAMP5E                 (1 << 4)              /* Bit 4 */
#define TAMP_CR1_TAMP6E                 (1 << 5)              /* Bit 5 */
#define TAMP_CR1_TAMP7E                 (1 << 6)              /* Bit 6 */
#define TAMP_CR1_ITAMP1E                (1 << 16)             /* Bit 16 */
#define TAMP_CR1_ITAMP2E                (1 << 17)             /* Bit 17 */
#define TAMP_CR1_ITAMP3E                (1 << 18)             /* Bit 18 */
#define TAMP_CR1_ITAMP4E                (1 << 19)             /* Bit 19 */
#define TAMP_CR1_ITAMP5E                (1 << 20)             /* Bit 20 */
#define TAMP_CR1_ITAMP6E                (1 << 21)             /* Bit 21 */
#define TAMP_CR1_ITAMP7E                (1 << 22)             /* Bit 22 */
#define TAMP_CR1_ITAMP8E                (1 << 23)             /* Bit 23 */
#define TAMP_CR1_ITAMP9E                (1 << 24)             /* Bit 24 */
#define TAMP_CR1_ITAMP11E               (1 << 26)             /* Bit 26 */

/* TAMP CR2 register */

#define TAMP_CR2_TAMP1POM               (1 << 0)              /* Bit 0 */
#define TAMP_CR2_TAMP2POM               (1 << 1)              /* Bit 1 */
#define TAMP_CR2_TAMP3POM               (1 << 2)              /* Bit 2 */
#define TAMP_CR2_TAMP4POM               (1 << 3)              /* Bit 3 */
#define TAMP_CR2_TAMP5POM               (1 << 4)              /* Bit 4 */
#define TAMP_CR2_TAMP6POM               (1 << 5)              /* Bit 5 */
#define TAMP_CR2_TAMP7POM               (1 << 6)              /* Bit 6 */
#define TAMP_CR2_TAMP1MSK               (1 << 16)             /* Bit 16 */
#define TAMP_CR2_TAMP2MSK               (1 << 17)             /* Bit 17 */
#define TAMP_CR2_TAMP3MSK               (1 << 18)             /* Bit 18 */
#define TAMP_CR2_BKBLOCK                (1 << 22)             /* Bit 22 */
#define TAMP_CR2_BKERASE                (1 << 23)             /* Bit 23 */
#define TAMP_CR2_TAMP1TRG               (1 << 24)             /* Bit 24 */
#define TAMP_CR2_TAMP2TRG               (1 << 25)             /* Bit 25 */
#define TAMP_CR2_TAMP3TRG               (1 << 26)             /* Bit 26 */
#define TAMP_CR2_TAMP4TRG               (1 << 27)             /* Bit 27 */
#define TAMP_CR2_TAMP5TRG               (1 << 28)             /* Bit 28 */
#define TAMP_CR2_TAMP6TRG               (1 << 29)             /* Bit 29 */
#define TAMP_CR2_TAMP7TRG               (1 << 30)             /* Bit 30 */

/* TAMP CR3 register */

#define TAMP_CR3_ITAMP1POM              (1 << 0)              /* Bit 0 */
#define TAMP_CR3_ITAMP2POM              (1 << 1)              /* Bit 1 */
#define TAMP_CR3_ITAMP3POM              (1 << 2)              /* Bit 2 */
#define TAMP_CR3_ITAMP4POM              (1 << 3)              /* Bit 3 */
#define TAMP_CR3_ITAMP5POM              (1 << 4)              /* Bit 4 */
#define TAMP_CR3_ITAMP6POM              (1 << 5)              /* Bit 5 */
#define TAMP_CR3_ITAMP7POM              (1 << 6)              /* Bit 6 */
#define TAMP_CR3_ITAMP8POM              (1 << 7)              /* Bit 7 */
#define TAMP_CR3_ITAMP9POM              (1 << 8)              /* Bit 8 */
#define TAMP_CR3_ITAMP11POM             (1 << 10)             /* Bit 10 */

/* TAMP FLTCR register */

#define TAMP_FLTCR_TAMPFREQ_SHIFT       (0)                   /* Bits 0-2 */
#define TAMP_FLTCR_TAMPFREQ_MASK        (0x7 << TAMP_FLTCR_TAMPFREQ_SHIFT)
#define TAMP_FLTCR_TAMPFLT_SHIFT        (3)                   /* Bits 3-4 */
#define TAMP_FLTCR_TAMPFLT_MASK         (0x3 << TAMP_FLTCR_TAMPFLT_SHIFT)
#define TAMP_FLTCR_TAMPPRCH_SHIFT       (5)                   /* Bits 5-6 */
#define TAMP_FLTCR_TAMPPRCH_MASK        (0x3 << TAMP_FLTCR_TAMPPRCH_SHIFT)
#define TAMP_FLTCR_TAMPPUDIS            (1 << 7)              /* Bit 7 */

/* TAMP ATCR1 register */

#define TAMP_ATCR1_TAMP1AM              (1 << 0)              /* Bit 0 */
#define TAMP_ATCR1_TAMP2AM              (1 << 1)              /* Bit 1 */
#define TAMP_ATCR1_TAMP3AM              (1 << 2)              /* Bit 2 */
#define TAMP_ATCR1_TAMP4AM              (1 << 3)              /* Bit 3 */
#define TAMP_ATCR1_TAMP5AM              (1 << 4)              /* Bit 4 */
#define TAMP_ATCR1_TAMP6AM              (1 << 5)              /* Bit 5 */
#define TAMP_ATCR1_TAMP7AM              (1 << 6)              /* Bit 6 */
#define TAMP_ATCR1_ATOSEL1_SHIFT        (8)                   /* Bits 8-9 */
#define TAMP_ATCR1_ATOSEL1_MASK         (0x3 << TAMP_ATCR1_ATOSEL1_SHIFT)
#define TAMP_ATCR1_ATOSEL2_SHIFT        (10)                  /* Bits 10-11 */
#define TAMP_ATCR1_ATOSEL2_MASK         (0x3 << TAMP_ATCR1_ATOSEL2_SHIFT)
#define TAMP_ATCR1_ATOSEL3_SHIFT        (12)                  /* Bits 12-13 */
#define TAMP_ATCR1_ATOSEL3_MASK         (0x3 << TAMP_ATCR1_ATOSEL3_SHIFT)
#define TAMP_ATCR1_ATOSEL4_SHIFT        (14)                  /* Bits 14-15 */
#define TAMP_ATCR1_ATOSEL4_MASK         (0x3 << TAMP_ATCR1_ATOSEL4_SHIFT)
#define TAMP_ATCR1_ATCKSEL_SHIFT        (16)                  /* Bits 16-19 */
#define TAMP_ATCR1_ATCKSEL_MASK         (0xf << TAMP_ATCR1_ATCKSEL_SHIFT)
#define TAMP_ATCR1_ATPER_SHIFT          (24)                  /* Bits 24-26 */
#define TAMP_ATCR1_ATPER_MASK           (0x7 << TAMP_ATCR1_ATPER_SHIFT)
#define TAMP_ATCR1_ATOSHARE             (1 << 30)             /* Bit 30 */
#define TAMP_ATCR1_FLTEN                (1 << 31)             /* Bit 31 */

/* TAMP ATSEEDR register */

#define TAMP_ATSEEDR_SEED_SHIFT         (0)                   /* Bits 0-31 */
#define TAMP_ATSEEDR_SEED_MASK          (0xffffffff << TAMP_ATSEEDR_SEED_SHIFT)

/* TAMP ATOR register */

#define TAMP_ATOR_PRNG_SHIFT            (0)                   /* Bits 0-7 */
#define TAMP_ATOR_PRNG_MASK             (0xff << TAMP_ATOR_PRNG_SHIFT)
#define TAMP_ATOR_SEEDF                 (1 << 14)             /* Bit 14 */
#define TAMP_ATOR_INITS                 (1 << 15)             /* Bit 15 */

/* TAMP ATCR2 register */

#define TAMP_ATCR2_ATOSEL1_SHIFT        (8)                   /* Bits 8-10 */
#define TAMP_ATCR2_ATOSEL1_MASK         (0x7 << TAMP_ATCR2_ATOSEL1_SHIFT)
#define TAMP_ATCR2_ATOSEL2_SHIFT        (11)                  /* Bits 11-13 */
#define TAMP_ATCR2_ATOSEL2_MASK         (0x7 << TAMP_ATCR2_ATOSEL2_SHIFT)
#define TAMP_ATCR2_ATOSEL3_SHIFT        (14)                  /* Bits 14-16 */
#define TAMP_ATCR2_ATOSEL3_MASK         (0x7 << TAMP_ATCR2_ATOSEL3_SHIFT)
#define TAMP_ATCR2_ATOSEL4_SHIFT        (17)                  /* Bits 17-19 */
#define TAMP_ATCR2_ATOSEL4_MASK         (0x7 << TAMP_ATCR2_ATOSEL4_SHIFT)
#define TAMP_ATCR2_ATOSEL5_SHIFT        (20)                  /* Bits 20-22 */
#define TAMP_ATCR2_ATOSEL5_MASK         (0x7 << TAMP_ATCR2_ATOSEL5_SHIFT)
#define TAMP_ATCR2_ATOSEL6_SHIFT        (23)                  /* Bits 23-25 */
#define TAMP_ATCR2_ATOSEL6_MASK         (0x7 << TAMP_ATCR2_ATOSEL6_SHIFT)
#define TAMP_ATCR2_ATOSEL7_SHIFT        (26)                  /* Bits 26-28 */
#define TAMP_ATCR2_ATOSEL7_MASK         (0x7 << TAMP_ATCR2_ATOSEL7_SHIFT)

/* TAMP SECCFGR register */

#define TAMP_SECCFGR_BKPRWSEC_SHIFT     (0)                   /* Bits 0-7 */
#define TAMP_SECCFGR_BKPRWSEC_MASK      (0xff << TAMP_SECCFGR_BKPRWSEC_SHIFT)
#define TAMP_SECCFGR_CNT1SEC            (1 << 15)             /* Bit 15 */
#define TAMP_SECCFGR_BKPWSEC_SHIFT      (16)                  /* Bits 16-23 */
#define TAMP_SECCFGR_BKPWSEC_MASK       (0xff << TAMP_SECCFGR_BKPWSEC_SHIFT)
#define TAMP_SECCFGR_BHKLOCK            (1 << 30)             /* Bit 30 */
#define TAMP_SECCFGR_TAMPSEC            (1 << 31)             /* Bit 31 */

/* TAMP PRIVCFGR register */

#define TAMP_PRIVCFGR_CNT1PRIV          (1 << 15)             /* Bit 15 */
#define TAMP_PRIVCFGR_BKPRWPRIV         (1 << 29)             /* Bit 29 */
#define TAMP_PRIVCFGR_BKPWPRIV          (1 << 30)             /* Bit 30 */
#define TAMP_PRIVCFGR_TAMPPRIV          (1 << 31)             /* Bit 31 */

/* TAMP IER register */

#define TAMP_IER_TAMP1IE                (1 << 0)              /* Bit 0 */
#define TAMP_IER_TAMP2IE                (1 << 1)              /* Bit 1 */
#define TAMP_IER_TAMP3IE                (1 << 2)              /* Bit 2 */
#define TAMP_IER_TAMP4IE                (1 << 3)              /* Bit 3 */
#define TAMP_IER_TAMP5IE                (1 << 4)              /* Bit 4 */
#define TAMP_IER_TAMP6IE                (1 << 5)              /* Bit 5 */
#define TAMP_IER_TAMP7IE                (1 << 6)              /* Bit 6 */
#define TAMP_IER_ITAMP1IE               (1 << 16)             /* Bit 16 */
#define TAMP_IER_ITAMP2IE               (1 << 17)             /* Bit 17 */
#define TAMP_IER_ITAMP3IE               (1 << 18)             /* Bit 18 */
#define TAMP_IER_ITAMP4IE               (1 << 19)             /* Bit 19 */
#define TAMP_IER_ITAMP5IE               (1 << 20)             /* Bit 20 */
#define TAMP_IER_ITAMP6IE               (1 << 21)             /* Bit 21 */
#define TAMP_IER_ITAMP7IE               (1 << 22)             /* Bit 22 */
#define TAMP_IER_ITAMP8IE               (1 << 23)             /* Bit 23 */
#define TAMP_IER_ITAMP9IE               (1 << 24)             /* Bit 24 */
#define TAMP_IER_ITAMP11IE              (1 << 26)             /* Bit 26 */

/* TAMP SR register */

#define TAMP_SR_TAMP1F                  (1 << 0)              /* Bit 0 */
#define TAMP_SR_TAMP2F                  (1 << 1)              /* Bit 1 */
#define TAMP_SR_TAMP3F                  (1 << 2)              /* Bit 2 */
#define TAMP_SR_TAMP4F                  (1 << 3)              /* Bit 3 */
#define TAMP_SR_TAMP5F                  (1 << 4)              /* Bit 4 */
#define TAMP_SR_TAMP6F                  (1 << 5)              /* Bit 5 */
#define TAMP_SR_TAMP7F                  (1 << 6)              /* Bit 6 */
#define TAMP_SR_ITAMP1F                 (1 << 16)             /* Bit 16 */
#define TAMP_SR_ITAMP2F                 (1 << 17)             /* Bit 17 */
#define TAMP_SR_ITAMP3F                 (1 << 18)             /* Bit 18 */
#define TAMP_SR_ITAMP4F                 (1 << 19)             /* Bit 19 */
#define TAMP_SR_ITAMP5F                 (1 << 20)             /* Bit 20 */
#define TAMP_SR_ITAMP6F                 (1 << 21)             /* Bit 21 */
#define TAMP_SR_ITAMP7F                 (1 << 22)             /* Bit 22 */
#define TAMP_SR_ITAMP8F                 (1 << 23)             /* Bit 23 */
#define TAMP_SR_ITAMP9F                 (1 << 24)             /* Bit 24 */
#define TAMP_SR_ITAMP11F                (1 << 26)             /* Bit 26 */

/* TAMP MISR register */

#define TAMP_MISR_TAMP1MF               (1 << 0)              /* Bit 0 */
#define TAMP_MISR_TAMP2MF               (1 << 1)              /* Bit 1 */
#define TAMP_MISR_TAMP3MF               (1 << 2)              /* Bit 2 */
#define TAMP_MISR_TAMP4MF               (1 << 3)              /* Bit 3 */
#define TAMP_MISR_TAMP5MF               (1 << 4)              /* Bit 4 */
#define TAMP_MISR_TAMP6MF               (1 << 5)              /* Bit 5 */
#define TAMP_MISR_TAMP7MF               (1 << 6)              /* Bit 6 */
#define TAMP_MISR_ITAMP1MF              (1 << 16)             /* Bit 16 */
#define TAMP_MISR_ITAMP2MF              (1 << 17)             /* Bit 17 */
#define TAMP_MISR_ITAMP3MF              (1 << 18)             /* Bit 18 */
#define TAMP_MISR_ITAMP4MF              (1 << 19)             /* Bit 19 */
#define TAMP_MISR_ITAMP5MF              (1 << 20)             /* Bit 20 */
#define TAMP_MISR_ITAMP6MF              (1 << 21)             /* Bit 21 */
#define TAMP_MISR_ITAMP7MF              (1 << 22)             /* Bit 22 */
#define TAMP_MISR_ITAMP8MF              (1 << 23)             /* Bit 23 */
#define TAMP_MISR_ITAMP9MF              (1 << 24)             /* Bit 24 */
#define TAMP_MISR_ITAMP11MF             (1 << 26)             /* Bit 26 */

/* TAMP SMISR register */

#define TAMP_SMISR_TAMP1MF              (1 << 0)              /* Bit 0 */
#define TAMP_SMISR_TAMP2MF              (1 << 1)              /* Bit 1 */
#define TAMP_SMISR_TAMP3MF              (1 << 2)              /* Bit 2 */
#define TAMP_SMISR_TAMP4MF              (1 << 3)              /* Bit 3 */
#define TAMP_SMISR_TAMP5MF              (1 << 4)              /* Bit 4 */
#define TAMP_SMISR_TAMP6MF              (1 << 5)              /* Bit 5 */
#define TAMP_SMISR_TAMP7MF              (1 << 6)              /* Bit 6 */
#define TAMP_SMISR_ITAMP1MF             (1 << 16)             /* Bit 16 */
#define TAMP_SMISR_ITAMP2MF             (1 << 17)             /* Bit 17 */
#define TAMP_SMISR_ITAMP3MF             (1 << 18)             /* Bit 18 */
#define TAMP_SMISR_ITAMP4MF             (1 << 19)             /* Bit 19 */
#define TAMP_SMISR_ITAMP5MF             (1 << 20)             /* Bit 20 */
#define TAMP_SMISR_ITAMP6MF             (1 << 21)             /* Bit 21 */
#define TAMP_SMISR_ITAMP7MF             (1 << 22)             /* Bit 22 */
#define TAMP_SMISR_ITAMP8MF             (1 << 23)             /* Bit 23 */
#define TAMP_SMISR_ITAMP9MF             (1 << 24)             /* Bit 24 */
#define TAMP_SMISR_ITAMP11MF            (1 << 26)             /* Bit 26 */

/* TAMP SCR register */

#define TAMP_SCR_CTAMP1F                (1 << 0)              /* Bit 0 */
#define TAMP_SCR_CTAMP2F                (1 << 1)              /* Bit 1 */
#define TAMP_SCR_CTAMP3F                (1 << 2)              /* Bit 2 */
#define TAMP_SCR_CTAMP4F                (1 << 3)              /* Bit 3 */
#define TAMP_SCR_CTAMP5F                (1 << 4)              /* Bit 4 */
#define TAMP_SCR_CTAMP6F                (1 << 5)              /* Bit 5 */
#define TAMP_SCR_CTAMP7F                (1 << 6)              /* Bit 6 */
#define TAMP_SCR_CTAMP8F                (1 << 7)              /* Bit 7 */
#define TAMP_SCR_CITAMP1F               (1 << 16)             /* Bit 16 */
#define TAMP_SCR_CITAMP2F               (1 << 17)             /* Bit 17 */
#define TAMP_SCR_CITAMP3F               (1 << 18)             /* Bit 18 */
#define TAMP_SCR_CITAMP4F               (1 << 19)             /* Bit 19 */
#define TAMP_SCR_CITAMP5F               (1 << 20)             /* Bit 20 */
#define TAMP_SCR_CITAMP6F               (1 << 21)             /* Bit 21 */
#define TAMP_SCR_CITAMP7F               (1 << 22)             /* Bit 22 */
#define TAMP_SCR_CITAMP8F               (1 << 23)             /* Bit 23 */
#define TAMP_SCR_CITAMP9F               (1 << 24)             /* Bit 24 */
#define TAMP_SCR_CITAMP11F              (1 << 26)             /* Bit 26 */

/* TAMP COUNT1R register */

#define TAMP_COUNT1R_COUNT_SHIFT        (0)                   /* Bits 0-31 */
#define TAMP_COUNT1R_COUNT_MASK         (0xffffffff << TAMP_COUNT1R_COUNT_SHIFT)

/* TAMP OR register */

#define TAMP_OR_VCOREMEN                (1 << 0)              /* Bit 0 */

/* TAMP RPCFGR register */

#define TAMP_RPCFGR_RPCFG0              (1 << 0)              /* Bit 0 */

/* Generic alarm register field names (ALRMAR and ALRMBR are identical) */

#define RTC_ALRMR_SU_SHIFT              RTC_ALRMAR_SU_SHIFT
#define RTC_ALRMR_SU_MASK               RTC_ALRMAR_SU_MASK
#define RTC_ALRMR_ST_SHIFT              RTC_ALRMAR_ST_SHIFT
#define RTC_ALRMR_ST_MASK               RTC_ALRMAR_ST_MASK
#define RTC_ALRMR_MSK1                  RTC_ALRMAR_MSK1
#define RTC_ALRMR_MNU_SHIFT             RTC_ALRMAR_MNU_SHIFT
#define RTC_ALRMR_MNU_MASK              RTC_ALRMAR_MNU_MASK
#define RTC_ALRMR_MNT_SHIFT             RTC_ALRMAR_MNT_SHIFT
#define RTC_ALRMR_MNT_MASK              RTC_ALRMAR_MNT_MASK
#define RTC_ALRMR_MSK2                  RTC_ALRMAR_MSK2
#define RTC_ALRMR_HU_SHIFT              RTC_ALRMAR_HU_SHIFT
#define RTC_ALRMR_HU_MASK               RTC_ALRMAR_HU_MASK
#define RTC_ALRMR_HT_SHIFT              RTC_ALRMAR_HT_SHIFT
#define RTC_ALRMR_HT_MASK               RTC_ALRMAR_HT_MASK
#define RTC_ALRMR_PM                    RTC_ALRMAR_PM
#define RTC_ALRMR_MSK3                  RTC_ALRMAR_MSK3
#define RTC_ALRMR_DU_SHIFT              RTC_ALRMAR_DU_SHIFT
#define RTC_ALRMR_DU_MASK               RTC_ALRMAR_DU_MASK
#define RTC_ALRMR_DT_SHIFT              RTC_ALRMAR_DT_SHIFT
#define RTC_ALRMR_DT_MASK               RTC_ALRMAR_DT_MASK
#define RTC_ALRMR_WDSEL                 RTC_ALRMAR_WDSEL
#define RTC_ALRMR_MSK4                  RTC_ALRMAR_MSK4

/* Sub-second value, 16 bits in BCD mode */

#define RTC_SSR_MASK                    (0xffff)

/* Date register bits that must be written as zero */

#define RTC_DR_RESERVED_BITS            (0xff0000c0)

/* Write protection key sequence */

#define RTC_WPR_KEY1                    0xca
#define RTC_WPR_KEY2                    0x53

/* The backup registers live in TAMP on this RTC version */

#define STM32_RTC_BKR(n)                STM32_TAMP_BKPR(n)

#endif /* __ARCH_ARM_SRC_COMMON_STM32_HARDWARE_STM32_RTCC_V3_H */
