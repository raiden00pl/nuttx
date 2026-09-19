================
ST STM32N6570-DK
================

.. tags:: chip:stm32, chip:stm32n6, chip:stm32n657

The STM32N6570-DK Discovery kit is based on the STM32N657X0H3Q
Arm Cortex-M55 microcontroller.

Loading and running
===================

Set SW1 (BOOT1) to 1 and press RESET (B1) before programming.
Use STM32CubeProgrammer's signing tool to add the boot ROM image header:

.. code:: console

   $ STM32_SigningTool_CLI -bin nuttx.bin -nk -of 0x80000000 \
         -t fsbl -hv 2.3 -align -s -o nuttx-boot.bin

Program the resulting image into the on-board external flash, using the
loader shipped with STM32CubeProgrammer (adjust its path as needed):

.. code:: console

   $ STM32_Programmer_CLI -c port=SWD mode=UR \
         -el /path/to/MX66UW1G45G_STM32N6570-DK.stldr \
         -d nuttx-boot.bin 0x70000000 -v

Set SW1 (BOOT1) and SW2 (BOOT0) to 0, then reset or power-cycle the board.
The boot ROM loads NuttX from flash and starts it without a debugger or
separate FSBL.  The console is the ST-LINK Virtual COM Port at 115200 8N1.

The boot image header and padding occupy the first 1 KiB at
``0x34180000``; NuttX is linked at ``0x34180400``.  The boot ROM download
buffer limits the NuttX binary to 511 KiB, enforced by the linker script.
The ``-nk`` image is for a device that permits unauthenticated boot.

Configurations
==============

nsh
---

Minimal NuttShell configuration.
