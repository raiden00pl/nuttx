/****************************************************************************
 * arch/x86_64/src/intel64/intel64_cpustart.c
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

#include <assert.h>

#include <arch/arch.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>

#include "init/init.h"

#include "intel64_lowsetup.h"
#include "intel64_cpu.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile static int g_cpu_cntr = 1;
volatile static spinlock_t g_ap_boot;

/****************************************************************************
 * External functions
 ****************************************************************************/

extern void __ap_entry(void);
extern int up_pause_handler(int irq, void *c, void *arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_cpu_ready
 *
 * Description:
 *   Get CPU ready fglag
 *
 ****************************************************************************/

static int up_cpu_ready(int cpu)
{
  struct intel64_cpu_s *priv  = &g_cpu_priv[cpu];
  bool                  ready = false;

  spin_lock(&g_ap_boot);
  ready = priv->ready;
  spin_unlock(&g_ap_boot);

  return ready;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_cpu_boot
 *
 * Description:
 *   Boot handler for AP core[x]
 *
 * Input Parameters:
 *   cpu - The index of the CPU being started.  This will be a numeric
 *         value in the range of one to (CONFIG_SMP_NCPUS-1).
 *         (CPU 0 is already active)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_ap_boot(int cpu)
{
  struct tss_s *tss = NULL;

  spin_lock(&g_ap_boot);

  /* Do some checking on CPU compatibilities at the top of this function */

  x86_64_check_and_enable_capability();

  /* Low-level, pre-OS initialization */

  intel64_lowsetup();

  /* Configure interrupts */

  up_irqinitialize();

  /* Get TSS - one per CPU */

  tss = (struct tss_s *)((uintptr_t)&g_tss64_low + X86_64_LOAD_OFFSET +
                         (X86_TSS_SIZE * cpu));

  /* Store CPU private data in TSS */

  tss->cpu = &g_cpu_priv[cpu];

#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Notify that this CPU has started */

  sched_note_cpu_started(this_task());
#endif

  /* Connect Pause IRQ to CPU */

  irq_attach(SMP_IPI_IRQ, up_pause_handler, NULL);
  up_enable_irq(SMP_IPI_IRQ);

  /* CPU ready */

  g_cpu_priv[cpu].ready = true;
  g_cpu_cntr += 1;

  spin_unlock(&g_ap_boot);

  if (g_cpu_cntr >= CONFIG_SMP_NCPUS)
    {
      /* Revoke the lower memory */

      __revoke_low_memory();
    }

  /* Then transfer control to the IDLE task */

  nx_idle_trampoline();
}

/****************************************************************************
 * Name: up_ap_startup
 *
 * Description:
 *   Startup AP CPU
 *
 ****************************************************************************/

int up_ap_startup(int cpu)
{
  uint64_t dest   = 0;
  uint64_t vect   = 0;
  uint64_t regval = 0;

  /* Get destination */

  dest = MSR_X2APIC_DESTINATION((uint64_t)cpu);

  /* Copy the AP trampoline to a fixed address */

  vect = (uint32_t)((uintptr_t)&__ap_entry) >> 12;

  /* Send an INIT IPI to the CPU */

  regval = MSR_X2APIC_ICR_INIT | dest;
  write_msr(MSR_X2APIC_ICR, regval);

  /* Wait for 10 ms */

  up_mdelay(10);

  /* Send an STARTUP IPI to the CPU */

  regval = MSR_X2APIC_ICR_STARTUP | dest | vect;
  write_msr(MSR_X2APIC_ICR, regval);

  /* Wait for 1 ms */

  up_mdelay(10);

  /* Check CPU ready flag */

  if (up_cpu_ready(cpu) == false)
    {
      /* Send another STARTUP IPI to the CPU */

      regval = MSR_X2APIC_ICR_STARTUP | dest | vect;
      write_msr(MSR_X2APIC_ICR, regval);

      /* Wait for 1 ms */

      up_mdelay(1);

      /* Check CPU flag again */

      if (up_cpu_ready(cpu) == false)
        {
          /* Raise assertion if we fail */

          ASSERT(0);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: up_cpu_start
 *
 * Description:
 *   In an SMP configuration, only one CPU is initially active (CPU 0).
 *   System initialization occurs on that single thread. At the completion of
 *   the initialization of the OS, just before beginning normal multitasking,
 *   the additional CPUs would be started by calling this function.
 *
 *   Each CPU is provided the entry point to its IDLE task when started.  A
 *   TCB for each CPU's IDLE task has been initialized and placed in the
 *   CPU's g_assignedtasks[cpu] list.  No stack has been allocated or
 *   initialized.
 *
 *   The OS initialization logic calls this function repeatedly until each
 *   CPU has been started, 1 through (CONFIG_SMP_NCPUS-1).
 *
 * Input Parameters:
 *   cpu - The index of the CPU being started.  This will be a numeric
 *         value in the range of one to (CONFIG_SMP_NCPUS-1).
 *         (CPU 0 is already active)
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_cpu_start(int cpu)
{
  int ret = OK;

  if (cpu != 0)
    {
      /* Startup AP core */

      ret = up_ap_startup(cpu);
    }

  return ret;
}
