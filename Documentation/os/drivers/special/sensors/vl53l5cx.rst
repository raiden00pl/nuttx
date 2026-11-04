========
VL53L5CX
========

The ST VL53L5CX is a multizone time-of-flight sensor with 8x8 or 4x4
zones on I2C (address 0x29).  The driver (``CONFIG_SENSORS_VL53L5CX``)
is a V4L2 depth capture device, see :doc:`/os/drivers/special/video`.

Application Programming Interface
=================================

.. code-block:: c

   #include <nuttx/sensors/vl53l5cx.h>

   int vl53l5cx_register(FAR const char *devpath,
                         FAR struct i2c_master_s *i2c, uint8_t addr);

The board registers the device, usually at ``CONFIG_VL53L5CX_DEVPATH``
(``/dev/video1``).  The registration checks the device ID.  The I2C
frequency is ``CONFIG_VL53L5CX_I2C_FREQUENCY``.

Frames
======

The only format is ``V4L2_PIX_FMT_Z16``: one little-endian 16-bit
distance in millimetres per zone, in the sensor's zone order (row-major;
the optics mirror the scene).  A zone is 0 when it has no target or its
target status is not 5 or 9 (range valid).  The sensor reports one target
per zone and ranges continuously.

=====  ==========  ==============
Size   Frame rate  Default
=====  ==========  ==============
8x8    1-15 Hz     yes, at 15 Hz
4x4    1-60 Hz
=====  ==========  ==============

The frame intervals are enumerated per frame size
(``VIDIOC_ENUM_FRAMEINTERVALS`` with the width and height set) and must
be a whole number of frames per second.  A frame that fails the integrity
check is returned with ``V4L2_BUF_FLAG_ERROR``.  Frames are dropped while
no buffer is queued.

The first ``VIDIOC_STREAMON`` boots the sensor and loads the files below
(1-2.5 s); it fails with ``-ENOENT`` if a file is missing and
``-EINVAL`` if its size is wrong.  ``VIDIOC_STREAMOFF`` stops ranging,
the last close puts the sensor to sleep.  The driver polls the sensor on
the low priority work queue; the INT pin is not used.

Files
=====

The sensor needs four binary files from ST's VL53L5CX Ultra Lite Driver
(ULD).  They are not part of NuttX.  The firmware and the configuration
must come from the same ULD release.

.. list-table::
   :header-rows: 1

   * - Kconfig (default path)
     - ULD array
     - Bytes
   * - ``CONFIG_VL53L5CX_FIRMWARE_PATH`` (``/mnt/vl53l5cx_fw.bin``)
     - ``VL53L5CX_FIRMWARE``
     - 86016
   * - ``CONFIG_VL53L5CX_CONFIG_PATH`` (``/mnt/vl53l5cx_cfg.bin``)
     - ``VL53L5CX_DEFAULT_CONFIGURATION``
     - 972
   * - ``CONFIG_VL53L5CX_XTALK_PATH`` (``/mnt/vl53l5cx_xtalk.bin``)
     - ``VL53L5CX_DEFAULT_XTALK``
     - 776
   * - ``CONFIG_VL53L5CX_NVMREQ_PATH`` (``/mnt/vl53l5cx_nvm.bin``)
     - ``VL53L5CX_GET_NVM_CMD``
     - 40

The arrays are in ``vl53l5cx_buffers.h`` of the ULD.  To make a file, take
the elements of the array between ``{`` and ``};``, drop the comments,
and write each hexadecimal element as one byte, in array order.  Element
107 of the configuration is the symbol ``VL53L5CX_FW_NBTAR_RANGING``
instead of a number: write any byte there (``0x02``), the driver sets it
when it loads the file.  Copy the files to the paths above, for example
on a microSD card mounted at ``/mnt``.
