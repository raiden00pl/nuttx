================
ST Nucleo-H533RE
================

.. tags:: chip:stm32, chip:stm32h5, chip:stm32h533

Board Information
=================

This page discusses issues unique to NuttX configurations for the
STMicro NUCLEO-H533RE development board featuring the STM32H533RE
MCU. The STM32H533RE is a 250MHz Cortex-M33 operation with 512KBytes Flash
memory and 272KByte SRAM. The board features:

- On-board ST-LINK-V3EC for programming and debugging
- 1 user LED
- Two pushbuttons (user and reset)
- 32.768 kHz crystal oscillator
- USB Type-C connector
- Board connectors:

  - USB-C
  - Arduino Uno V3 connector
  - ST morpho

Refer to the http://www.st.com website for further information about this
board (search keyword: NUCLEO-H533RE)

Serial Console
==============

The Nucleo Virtual Console is used as the default serial console in all
configurations. It uses Serial Port 2 (USART2) with TX on PA2 and RX on
PA3.

   ================= ===
   VCOM Signal       Pin
   ================= ===
   SERIAL_RX         PA3
   SERIAL_TX         PA2
   ================= ===

These signals are internally connected to the on board ST-Link.

LEDs
====

The Nucleo-H533RE has one user controllable LED, LD2 a Green LED connected
to PA5. If CONFIG_ARCH_LEDS is defined, this LED is used to encode
OS-related events:

  ==================  =======================  =========
  SYMBOL              Meaning                  LED state
  ==================  =======================  =========
  LED_STARTED         NuttX has been started   OFF
  LED_HEAPALLOCATE    Heap has been allocated  OFF
  LED_IRQSENABLED     Interrupts enabled       OFF
  LED_STACKCREATED    Idle stack created       ON
  LED_INIRQ           In an interrupt          N/C
  LED_SIGNAL          In a signal handler      N/C
  LED_ASSERTION       An assertion failed      N/C
  LED_PANIC           The system has crashed   Blinking
  ==================  =======================  =========

Pushbuttons
===========

B1 USER: the user button is connected to the I/O PC13 of the STM32
microcontroller.

Configurations
==============

nsh
---

Configures the NuttShell (nsh) located at apps/examples/nsh. This
configuration enables a serial console on USART2 (Nucleo Virtual Console).
