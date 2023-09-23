/****************************************************************************
 * boards/arm/nrf52/thingy52/src/nrf52_sensors.c
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
#include <sys/types.h>
#include <syslog.h>

#ifdef CONFIG_SENSORS_BH1745NUC
#  include <nrf52_bh1745nuc.h>
#endif

#include "thingy52.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_sensors_init
 *
 * Description:
 *   Initialzie on-board sensors
 *
 ****************************************************************************/

int nrf52_sensors_init(void)
{
  int ret = OK;

  UNUSED(ret);

#ifdef CONFIG_SENSORS_BH1745NUC
  ret = nrf52_bh1745nuc_init(0, 0, BH1745NUC_I2C_ADDR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: nrf52_bh1745nuc_init() failed: %d\n", ret);
    }
#endif

  return ret;
}
