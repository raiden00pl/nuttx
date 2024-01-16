/****************************************************************************
 * arch/x86_64/src/intel64/intel64_cpu.c
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/arch.h>
#include <arch/irq.h>
#include <arch/io.h>
#include <arch/acpi.h>

#include <assert.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* CPU private data */

struct intel64_cpu_s g_cpu_priv[CONFIG_SMP_NCPUS];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_cpu_init
 *
 * Description:
 *   Initialize CPU data.
 *
 ****************************************************************************/

void up_cpu_init(void)
{
  struct acpi_lapic_s *lapic = NULL;
  int                  i     = 0;
  int                  ret   = OK;

  /* Map logical CPU to Local APIC IDs */

  for (i = 0; i < CONFIG_SMP_NCPUS; i += 1)
    {
      ret = acpi_lapic_get(i, &lapic);
      if (ret == OK)
        {
          g_cpu_priv[i].loapic_id = lapic->apic_id;
          g_cpu_priv[i].id        = i;
          g_cpu_priv[i].ready     = false;
        }
      else
        {
          /* We want to fail early when NCPUS doesn't match the number
           * of availalbe CPUs
           */

          /* Paic if not found */

          PANIC();
        }
    }
}

/****************************************************************************
 * Name: up_lopaic_to_cpu
 *
 * Description:
 *   Get CPU index for a given Local APIC ID
 *
 ****************************************************************************/

uint8_t up_loapic_to_cpu(uint8_t loapic)
{
  int i = 0;

  for (i = 0; i < CONFIG_SMP_NCPUS; i += 1)
    {
      if (g_cpu_priv[i].loapic_id == loapic)
        {
          return i;
        }
    }

  /* Paic if not found */

  PANIC();
}

/****************************************************************************
 * Name: up_cpu_to_loapic
 *
 * Description:
 *   Get Local APIC ID for a given CPU index
 *
 ****************************************************************************/

uint8_t up_cpu_to_loapic(uint8_t cpu)
{
  return g_cpu_priv[cpu].loapic_id;
}
