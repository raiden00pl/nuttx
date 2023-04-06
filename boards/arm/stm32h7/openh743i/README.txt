README
======

SDMMC
-----

The SDMMC1 interface seems not to work correctly with clock frequencies higier
than 20 MHz.

USBHS
-----

The reset pin on the ULPU port must be connected to the PE2 pin of the MCU.
If it's not connected, ULPI interface may not work correctly without power reset.
