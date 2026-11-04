==========
ST STM32N6
==========

This is a port of NuttX to the STM32N6 family.
The STM32N6 is a chip based on the Arm Cortex-M55.

Supported MCUs
==============

===========  ======= ================
MCU          Support Note
===========  ======= ================
STM32N645     No
STM32N647     No
STM32N655     No
STM32N657     Yes    STM32N657X0 only
===========  ======= ================

Peripheral Support
==================

The following list indicates peripherals supported in NuttX:

==========  =======  ===================================================
Peripheral  Support  Notes
==========  =======  ===================================================
ADC         Yes      ADC1 and ADC2 through the common ADC v3 driver,
                     interrupt-driven conversions only (no DMA or timer
                     trigger).
Cache       Yes      L1 I- and D-cache (``CONFIG_ARMV8M_ICACHE``,
                     ``CONFIG_ARMV8M_DCACHE``).  The USB host driver uses
                     the slave (CPU FIFO) mode and needs no maintenance.
DCMIPP      Yes      Pipe1 with the CSI-2 host as a V4L2 capture
                     interface: raw Bayer to RGB565 (black level, white
                     balance, demosaicing, colour matrix, gamma) and
                     scaling.  Automatic exposure and grey world white
                     balance from the pipe1 statistics
                     (``CONFIG_STM32_DCMIPP_AE``,
                     ``CONFIG_STM32_DCMIPP_AWB``), switched with
                     ``V4L2_CID_EXPOSURE_AUTO`` and
                     ``V4L2_CID_AUTO_WHITE_BALANCE``.  No pipe0/pipe2 or
                     parallel input.
DMA         Yes      GPDMA1 and HPDMA1 through the common GPDMA driver.
                     The channels are Secure and privileged.
DMA2D       Yes      Common DMA2D driver as LTDC framebuffer overlays
                     (fill, blit, blend).  The bus master is made Secure
                     and privileged through the RIF.
ETH         Yes      ETH1 through the common Ethernet v2 driver, MII, RMII
                     or RGMII up to 1000 Mbps.
EXTI        Yes      GPIO interrupts through the common driver.
GPIO        Yes
I2C         Yes      I2C1-4 through the common I2C v2 driver.
IWDG        Yes      Through the common driver.
LPTIM       Yes      Through the common driver.
LTDC        Yes      Common LTDC driver (v2 register layout), RGB565,
                     RGB888 and ARGB8888 layers.  The layer bus masters
                     are made Secure and privileged through the RIF.
MDF         Yes      MDF1 filter 0 as an audio capture device for a PDM
                     microphone (Sinc4, reshape and high-pass filters,
                     GPDMA, 16-bit PCM at 8, 16, 32 or 48 kHz).
MPU         Yes      Armv8-M MPU (``CONFIG_ARM_MPU``).  The STM32N6570-DK
                     maps its framebuffers non-cacheable with it.
PWR         Yes      Supply configuration, VOS, I/O supply validity.
RCC         Yes      HSI/HSE, PLL1-4, IC1-20, bus prescalers and kernel
                     clock selection from board.h.
RNG         Yes      Through the common driver.
RTC         Yes      Calendar, sub-seconds, Alarm A/B and wakeup timer
                     through the common RTC v3 driver.
SAI         Yes      SAI1 through the common SAI driver, I2S master with
                     GPDMA.
SDMMC       Yes      SDMMC1-2 through the common IDMA SDMMC driver.  The
                     IDMA masters get Secure, privileged CID 1 RIF
                     attributes at start-up.
SPI         Yes      SPI1-6 through the common SPI v5 driver, master
                     mode, optional GPDMA.
TIM         Yes      Timer, PWM and tickless through the common drivers.
USART       Yes      USART1 and USART2.
USB         Yes      USB1 or USB2 OTG HS host through the common OTG v2
                     driver, full and low speed on the embedded HS PHY.
XSPI        Yes      XSPI1-2 and the XSPIM through the common XSPI v1
                     driver: indirect polled transfers (up to 16 data
                     lines, DTR, DQS) and memory-mapped mode.

NPU         No
USB device  No
==========  =======  ===================================================

References
==========

[RM0486] STMicroelectronics, STM32N647/657xx Arm®-based 32-bit MCUs

Supported Boards
================

.. toctree::
   :glob:
   :maxdepth: 1

   boards/*/*
