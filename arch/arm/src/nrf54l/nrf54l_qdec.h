/****************************************************************************
 * arch/arm/src/nrf54l/nrf54l_qdec.h
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

#ifndef __ARCH_ARM_SRC_NRF54L_NRF54L_QENCODER_H
#define __ARCH_ARM_SRC_NRF54L_NRF54L_QENCODER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct nrf54l_qeconfig_s
{
  uint8_t  sample_period;
  uint8_t  report_period;
  bool     enable_debounce;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct qe_lowerhalf_s;
struct qe_lowerhalf_s *
nrf54l_qeinitialize(int qdec, const struct nrf54l_qeconfig_s *config);

#endif /* __ARCH_ARM_SRC_NRF54L_NRF54L_QENCODER_H */
