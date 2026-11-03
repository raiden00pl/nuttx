/****************************************************************************
 * arch/arm/src/stm32n6/hardware/stm32n6xxx_mdf.h
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

#ifndef __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_MDF_H
#define __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_MDF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/stm32_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

/* Common registers */

#define STM32_MDF_GCR_OFFSET       0x0000  /* Global control */
#define STM32_MDF_CKGCR_OFFSET     0x0004  /* Clock generator control */
#define STM32_MDF_OR_OFFSET        0x0020  /* Option register */

/* Filter x (x = 0..5) registers, relative to the filter block */

#define STM32_MDF_FLT_OFFSET(x)    (0x0080 * ((x) + 1))

#define STM32_MDF_SITFCR_OFFSET    0x0000  /* Serial interface control */
#define STM32_MDF_BSMXCR_OFFSET    0x0004  /* Bitstream matrix control */
#define STM32_MDF_DFLTCR_OFFSET    0x0008  /* Digital filter control */
#define STM32_MDF_DFLTCICR_OFFSET  0x000c  /* MCIC configuration */
#define STM32_MDF_DFLTRSFR_OFFSET  0x0010  /* Reshape filter configuration */
#define STM32_MDF_DFLTINTR_OFFSET  0x0014  /* Integrator configuration */
#define STM32_MDF_OLDCR_OFFSET     0x0018  /* Out-of-limit detector control */
#define STM32_MDF_OLDTHLR_OFFSET   0x001c  /* OLD threshold low */
#define STM32_MDF_OLDTHHR_OFFSET   0x0020  /* OLD threshold high */
#define STM32_MDF_DLYCR_OFFSET     0x0024  /* Delay control */
#define STM32_MDF_SCDCR_OFFSET     0x0028  /* Short-circuit detector control */
#define STM32_MDF_DFLTIER_OFFSET   0x002c  /* Interrupt enable */
#define STM32_MDF_DFLTISR_OFFSET   0x0030  /* Interrupt status */
#define STM32_MDF_OECCR_OFFSET     0x0034  /* Offset error compensation */
#define STM32_MDF_SNPSDR_OFFSET    0x006c  /* Snapshot data */
#define STM32_MDF_DFLTDR_OFFSET    0x0070  /* Digital filter data */

/* Register Addresses *******************************************************/

#define STM32_MDF1_GCR             (STM32_MDF1_BASE + STM32_MDF_GCR_OFFSET)
#define STM32_MDF1_CKGCR           (STM32_MDF1_BASE + STM32_MDF_CKGCR_OFFSET)
#define STM32_MDF1_OR              (STM32_MDF1_BASE + STM32_MDF_OR_OFFSET)

#define STM32_MDF1_FLT_BASE(x)     (STM32_MDF1_BASE + STM32_MDF_FLT_OFFSET(x))

/* Register Bitfield Definitions ********************************************/

/* Global control register */

#define MDF_GCR_TRGO               (1 << 0)  /* Bit 0: Trigger output control */
#define MDF_GCR_ILVNB_SHIFT        (4)       /* Bits 4-7: Number of
                                              * interleaved filters - 1 */
#define MDF_GCR_ILVNB_MASK         (0xf << MDF_GCR_ILVNB_SHIFT)

/* Clock generator control register */

#define MDF_CKGCR_CKDEN            (1 << 0)  /* Bit 0: Kernel clock dividers enable */
#define MDF_CKGCR_CCK0EN           (1 << 1)  /* Bit 1: MDF_CCK0 clock enable */
#define MDF_CKGCR_CCK1EN           (1 << 2)  /* Bit 2: MDF_CCK1 clock enable */
#define MDF_CKGCR_CKGMOD           (1 << 4)  /* Bit 4: Start on trigger */
#define MDF_CKGCR_CCK0DIR          (1 << 5)  /* Bit 5: MDF_CCK0 is an output */
#define MDF_CKGCR_CCK1DIR          (1 << 6)  /* Bit 6: MDF_CCK1 is an output */
#define MDF_CKGCR_TRGSENS          (1 << 8)  /* Bit 8: Trigger on falling edge */
#define MDF_CKGCR_TRGSRC_SHIFT     (12)      /* Bits 12-15: Trigger source */
#define MDF_CKGCR_TRGSRC_MASK      (0xf << MDF_CKGCR_TRGSRC_SHIFT)
#define MDF_CKGCR_CCKDIV_SHIFT     (16)      /* Bits 16-19: MDF_CCKx divider
                                              * from the processing clock - 1 */
#define MDF_CKGCR_CCKDIV_MASK      (0xf << MDF_CKGCR_CCKDIV_SHIFT)
#define MDF_CKGCR_CCKDIV(n)        ((uint32_t)((n) - 1) << MDF_CKGCR_CCKDIV_SHIFT)
#define MDF_CKGCR_PROCDIV_SHIFT    (24)      /* Bits 24-30: Processing clock
                                              * divider from the kernel
                                              * clock - 1 */
#define MDF_CKGCR_PROCDIV_MASK     (0x7f << MDF_CKGCR_PROCDIV_SHIFT)
#define MDF_CKGCR_PROCDIV(n)       ((uint32_t)((n) - 1) << MDF_CKGCR_PROCDIV_SHIFT)
#define MDF_CKGCR_CCKACTIVE        (1 << 31) /* Bit 31: Clock generator active */

/* Serial interface control register */

#define MDF_SITFCR_SITFEN          (1 << 0)  /* Bit 0: Serial interface enable */
#define MDF_SITFCR_SCKSRC_SHIFT    (1)       /* Bits 1-2: Serial clock source */
#define MDF_SITFCR_SCKSRC_MASK     (3 << MDF_SITFCR_SCKSRC_SHIFT)
#  define MDF_SITFCR_SCKSRC_CCK0   (0 << MDF_SITFCR_SCKSRC_SHIFT)
#  define MDF_SITFCR_SCKSRC_CCK1   (1 << MDF_SITFCR_SCKSRC_SHIFT)
#  define MDF_SITFCR_SCKSRC_CKI    (2 << MDF_SITFCR_SCKSRC_SHIFT)
#define MDF_SITFCR_SITFMOD_SHIFT   (4)       /* Bits 4-5: Serial interface type:
                                              * LF master SPI, normal SPI
                                              * or Manchester (two
                                              * polarities) */
#define MDF_SITFCR_SITFMOD_MASK    (3 << MDF_SITFCR_SITFMOD_SHIFT)
#  define MDF_SITFCR_SITFMOD_LFSPI (0 << MDF_SITFCR_SITFMOD_SHIFT)
#  define MDF_SITFCR_SITFMOD_SPI   (1 << MDF_SITFCR_SITFMOD_SHIFT)
#  define MDF_SITFCR_SITFMOD_MANF  (2 << MDF_SITFCR_SITFMOD_SHIFT)
#  define MDF_SITFCR_SITFMOD_MANR  (3 << MDF_SITFCR_SITFMOD_SHIFT)
#define MDF_SITFCR_STH_SHIFT       (8)       /* Bits 8-12: Manchester symbol or
                                              * SPI clock absence threshold */
#define MDF_SITFCR_STH_MASK        (0x1f << MDF_SITFCR_STH_SHIFT)
#define MDF_SITFCR_STH(n)          ((uint32_t)(n) << MDF_SITFCR_STH_SHIFT)
#define MDF_SITFCR_SITFACTIVE      (1 << 31) /* Bit 31: Serial interface active */

/* Bitstream matrix control register */

#define MDF_BSMXCR_BSSEL_SHIFT     (0)       /* Bits 0-4: Bitstream selection */
#define MDF_BSMXCR_BSSEL_MASK      (0x1f << MDF_BSMXCR_BSSEL_SHIFT)

/* Bitstream n (0..5), sampled on the rising (f = 0) or falling (f = 1)
 * edge of the serial clock
 */

#define MDF_BSMXCR_BSSEL(n, f)     ((uint32_t)(((n) << 1) | (f)) << \
                                    MDF_BSMXCR_BSSEL_SHIFT)
#define MDF_BSMXCR_BSMXACTIVE      (1 << 31) /* Bit 31: Matrix active */

/* Digital filter control register */

#define MDF_DFLTCR_DFLTEN          (1 << 0)  /* Bit 0: Digital filter enable */
#define MDF_DFLTCR_DMAEN           (1 << 1)  /* Bit 1: DMA requests enable */
#define MDF_DFLTCR_FTH             (1 << 2)  /* Bit 2: RXFIFO threshold half full */
#define MDF_DFLTCR_ACQMOD_SHIFT    (4)       /* Bits 4-6: Acquisition mode:
                                              * asynchronous continuous or
                                              * single, synchronous
                                              * continuous or single, window
                                              * continuous, snapshot */
#define MDF_DFLTCR_ACQMOD_MASK     (7 << MDF_DFLTCR_ACQMOD_SHIFT)
#  define MDF_DFLTCR_ACQMOD_ACONT  (0 << MDF_DFLTCR_ACQMOD_SHIFT)
#  define MDF_DFLTCR_ACQMOD_ASING  (1 << MDF_DFLTCR_ACQMOD_SHIFT)
#  define MDF_DFLTCR_ACQMOD_SCONT  (2 << MDF_DFLTCR_ACQMOD_SHIFT)
#  define MDF_DFLTCR_ACQMOD_SSING  (3 << MDF_DFLTCR_ACQMOD_SHIFT)
#  define MDF_DFLTCR_ACQMOD_WCONT  (4 << MDF_DFLTCR_ACQMOD_SHIFT)
#  define MDF_DFLTCR_ACQMOD_SNAP   (5 << MDF_DFLTCR_ACQMOD_SHIFT)
#define MDF_DFLTCR_TRGSENS         (1 << 8)  /* Bit 8: Trigger on falling edge */
#define MDF_DFLTCR_TRGSRC_SHIFT    (12)      /* Bits 12-15: Trigger source */
#define MDF_DFLTCR_TRGSRC_MASK     (0xf << MDF_DFLTCR_TRGSRC_SHIFT)
#define MDF_DFLTCR_SNPSFMT         (1 << 16) /* Bit 16: Snapshot data format */
#define MDF_DFLTCR_NBDIS_SHIFT     (20)      /* Bits 20-27: Samples discarded
                                              * after the filter start */
#define MDF_DFLTCR_NBDIS_MASK      (0xff << MDF_DFLTCR_NBDIS_SHIFT)
#define MDF_DFLTCR_NBDIS(n)        ((uint32_t)(n) << MDF_DFLTCR_NBDIS_SHIFT)
#define MDF_DFLTCR_DFLTRUN         (1 << 30) /* Bit 30: Filter is running */
#define MDF_DFLTCR_DFLTACTIVE      (1 << 31) /* Bit 31: Filter active */

/* MCIC configuration register */

#define MDF_DFLTCICR_DATSRC_SHIFT  (0)       /* Bits 0-1: Filter data source:
                                              * bitstream matrix or ADC */
#define MDF_DFLTCICR_DATSRC_MASK   (3 << MDF_DFLTCICR_DATSRC_SHIFT)
#  define MDF_DFLTCICR_DATSRC_BSMX (0 << MDF_DFLTCICR_DATSRC_SHIFT)
#  define MDF_DFLTCICR_DATSRC_ADC  (2 << MDF_DFLTCICR_DATSRC_SHIFT)
#define MDF_DFLTCICR_CICMOD_SHIFT  (4)       /* Bits 4-6: CIC mode: two
                                              * filters with a FastSinc or
                                              * Sinc1-3 main filter, or one
                                              * Sinc4 or Sinc5 filter */
#define MDF_DFLTCICR_CICMOD_MASK   (7 << MDF_DFLTCICR_CICMOD_SHIFT)
#  define MDF_DFLTCICR_CICMOD_FASTSINC (0 << MDF_DFLTCICR_CICMOD_SHIFT)
#  define MDF_DFLTCICR_CICMOD_SINC1    (1 << MDF_DFLTCICR_CICMOD_SHIFT)
#  define MDF_DFLTCICR_CICMOD_SINC2    (2 << MDF_DFLTCICR_CICMOD_SHIFT)
#  define MDF_DFLTCICR_CICMOD_SINC3    (3 << MDF_DFLTCICR_CICMOD_SHIFT)
#  define MDF_DFLTCICR_CICMOD_SINC4    (4 << MDF_DFLTCICR_CICMOD_SHIFT)
#  define MDF_DFLTCICR_CICMOD_SINC5    (5 << MDF_DFLTCICR_CICMOD_SHIFT)
#define MDF_DFLTCICR_MCICD_SHIFT   (8)       /* Bits 8-16: CIC decimation
                                              * ratio - 1 */
#define MDF_DFLTCICR_MCICD_MASK    (0x1ff << MDF_DFLTCICR_MCICD_SHIFT)
#define MDF_DFLTCICR_MCICD(n)      ((uint32_t)((n) - 1) << MDF_DFLTCICR_MCICD_SHIFT)
#define MDF_DFLTCICR_SCALE_SHIFT   (20)      /* Bits 20-25: Scaling factor,
                                              * steps of about 3 dB */
#define MDF_DFLTCICR_SCALE_MASK    (0x3f << MDF_DFLTCICR_SCALE_SHIFT)

/* Reshape filter configuration register */

#define MDF_DFLTRSFR_RSFLTBYP      (1 << 0)  /* Bit 0: Reshape filter bypass */
#define MDF_DFLTRSFR_RSFLTD        (1 << 4)  /* Bit 4: Reshape filter decimation
                                              * by 1 instead of 4 */
#define MDF_DFLTRSFR_HPFBYP        (1 << 7)  /* Bit 7: High-pass filter bypass */
#define MDF_DFLTRSFR_HPFC_SHIFT    (8)       /* Bits 8-9: High-pass cut-off,
                                              * fraction of Fpcm */
#define MDF_DFLTRSFR_HPFC_MASK     (3 << MDF_DFLTRSFR_HPFC_SHIFT)
#  define MDF_DFLTRSFR_HPFC_0_000625 (0 << MDF_DFLTRSFR_HPFC_SHIFT)
#  define MDF_DFLTRSFR_HPFC_0_00125  (1 << MDF_DFLTRSFR_HPFC_SHIFT)
#  define MDF_DFLTRSFR_HPFC_0_0025   (2 << MDF_DFLTRSFR_HPFC_SHIFT)
#  define MDF_DFLTRSFR_HPFC_0_0095   (3 << MDF_DFLTRSFR_HPFC_SHIFT)

/* Integrator configuration register */

#define MDF_DFLTINTR_INTDIV_SHIFT  (0)       /* Bits 0-1: Integrator output
                                              * division */
#define MDF_DFLTINTR_INTDIV_MASK   (3 << MDF_DFLTINTR_INTDIV_SHIFT)
#define MDF_DFLTINTR_INTVAL_SHIFT  (4)       /* Bits 4-10: Integration value - 1 */
#define MDF_DFLTINTR_INTVAL_MASK   (0x7f << MDF_DFLTINTR_INTVAL_SHIFT)

/* Delay control register */

#define MDF_DLYCR_SKPDLY_SHIFT     (0)       /* Bits 0-6: Bitstream samples
                                              * to skip */
#define MDF_DLYCR_SKPDLY_MASK      (0x7f << MDF_DLYCR_SKPDLY_SHIFT)
#define MDF_DLYCR_SKPBF            (1 << 31) /* Bit 31: Skip busy flag */

/* Interrupt enable and status registers */

#define MDF_DFLTINT_FTH            (1 << 0)  /* Bit 0: RXFIFO threshold */
#define MDF_DFLTINT_DOVR           (1 << 1)  /* Bit 1: Data overflow */
#define MDF_DFLTINT_SSDR           (1 << 2)  /* Bit 2: Snapshot data ready */
#define MDF_DFLTINT_RXNE           (1 << 3)  /* Bit 3: RXFIFO not empty
                                              * (status only) */
#define MDF_DFLTINT_OLD            (1 << 4)  /* Bit 4: Out-of-limit detection */
#define MDF_DFLTINT_THL            (1 << 5)  /* Bit 5: Low threshold (status
                                              * only) */
#define MDF_DFLTINT_THH            (1 << 6)  /* Bit 6: High threshold (status
                                              * only) */
#define MDF_DFLTINT_SSOVR          (1 << 7)  /* Bit 7: Snapshot overrun */
#define MDF_DFLTINT_SCD            (1 << 8)  /* Bit 8: Short-circuit detection */
#define MDF_DFLTINT_SAT            (1 << 9)  /* Bit 9: Saturation detection */
#define MDF_DFLTINT_CKAB           (1 << 10) /* Bit 10: Clock absence detection */
#define MDF_DFLTINT_RFOVR          (1 << 11) /* Bit 11: Reshape filter overrun */
#define MDF_DFLTINT_ALL            (0x0fff)

/* Offset error compensation control register */

#define MDF_OECCR_OFFSET_SHIFT     (0)       /* Bits 0-25: Signed offset */
#define MDF_OECCR_OFFSET_MASK      (0x3ffffff << MDF_OECCR_OFFSET_SHIFT)

/* Digital filter data register: the 24-bit sample is left aligned */

#define MDF_DFLTDR_DR_SHIFT        (8)       /* Bits 8-31: Filter output data */
#define MDF_DFLTDR_DR_MASK         (0xffffff << MDF_DFLTDR_DR_SHIFT)

#endif /* __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_MDF_H */
