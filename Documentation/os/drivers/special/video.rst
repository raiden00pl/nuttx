====================
Video Device Drivers
====================

-  ``include/nuttx/video/``. All structures and APIs
   needed to work with video drivers are provided in this header
   file.

V4L2 capture
------------

``drivers/video/v4l2_cap.c`` implements ``/dev/videoN`` on top of an
image sensor (``imgsensor_register()``, ``include/nuttx/video/imgsensor.h``)
for the formats and the controls and an image data interface
(``imgdata_register()``, ``include/nuttx/video/imgdata.h``) that writes
the frames to memory.  ``V4L2_CID_GAIN`` is the sensor gain in driver
units, ``V4L2_CID_EXPOSURE_ABSOLUTE`` the exposure time in 100 us units.

A raw Bayer sensor reports its native format (for example
``V4L2_PIX_FMT_SRGGB10P``).  When the capture interface processes it, the
interface registers an image sensor of its own in front of the raw one
that reports the processed formats and sizes and forwards the other
controls; the raw sensor is not registered.

imx335
------

Sony IMX335 5 MP sensor (``CONFIG_VIDEO_IMX335``): I2C address 0x1a,
2592x1944 RAW10 RGGB on two MIPI CSI-2 lanes at 30 frames per second,
input clock 6, 18, 24, 27 or 74.25 MHz.  The register values come from the
Zephyr driver.  It supports ``V4L2_CID_EXPOSURE_ABSOLUTE`` and
``V4L2_CID_GAIN`` (0.3 dB steps, 0-240 for 0-72 dB); the initial values
are ``CONFIG_VIDEO_IMX335_EXPOSURE`` and ``CONFIG_VIDEO_IMX335_GAIN``.  It
needs an image signal processor such as the STM32N6 DCMIPP, which adds
automatic exposure and white balance.

max7456
-------

23 March 2019
Bill Gatliff <bgat@billgatliff.com>

The code in drivers/video/max7456.[ch] is a preliminary device driver for the MAX7456 analog
on-screen-display generator. This SPI slave chip is a popular feature in many
embedded devices due its low cost and power requirements. In particular, you
see it a lot on drone flight-management units.

I use the term "preliminary" because at present, only the most rudimentary
capabilities of the chip are supported:

 * chip reset and startup
 * read and write low-level chip control registers (DEBUG mode only)
 * write CA (Character Address) data to the chip's framebuffer memory

Some key missing features are, in no particular order:

 * VSYNC and HSYNC synchronization (prevents flicker)
 * ability to update NVM (define custom character sets)

If you have a factory-fresh chip, then the datasheet shows you what the factory
character data set looks like. If you've used the chip in other scenarios,
i.e. with Betaflight or similar, then your chip will almost certainly have had
the factory character data replaced with something application-specific.

Either way, you'll probably want to update your character set before long. I
should probably get that working, unless you want to take a look at it
yoruself...

The max7456_register() function starts things rolling. The omnibusf4 target
device provides an example (there may be others by the time you read this).

In normal use, the driver creates a set of interfaces under /dev, i.e.::

  /dev/osd0/fb
  /dev/osd0/raw   (*)
  /dev/osd0/vsync (*)

* - not yet implemented

By writing character data to the "fb" interface, you'll see data appear on the
display. NOTE that the data you write is NOT, for example, ASCII text: it is
the addresses of the characters in the chip's onboard character map.

For example, if entry 42 in your onboard character map is a bitmap that looks
like "H", then when you write the ASCII "*" (decimal 42, hex 2a), you'll see
that "H" appear on your screen.

If you build the code with the DEBUG macro defined, you will see a bunch more interfaces::

  /dev/osd0/VM0
  /dev/osd0/VM1
  /dev/osd/DMM
  ...
  ...

These are interfaces to the low-level chip registers, which can be read and/or
written to help you figure out what's going on inside the chip. They're
probably more useful for me than you, but there they are in case I'm wrong
about that.

b.g.

Depth (time-of-flight) cameras
------------------------------

Multizone and depth time-of-flight sensors produce an image and are V4L2
capture devices; single-zone sensors report one distance through the
sensor framework (``SENSOR_TYPE_PROXIMITY``).  The driver provides both
the ``imgsensor`` and ``imgdata`` ops and registers its own device with
``capture_register()``, so it does not affect the cameras.  The frame is
``V4L2_PIX_FMT_Z16``: one little-endian 16-bit distance in millimetres
per pixel, 0 when the pixel has no valid target.  The resolutions and
the frame rates are V4L2 frame sizes and frame intervals; an interval
entry that sets a size or a format only applies to it.

Driver: :doc:`sensors/vl53l5cx`.
