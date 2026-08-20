============================
``expat`` XML parser library
============================

``CONFIG_LIB_EXPAT`` provides the Expat streaming XML parser as a reusable
NuttX applications library.  The Make and CMake package definitions download
the version selected by ``CONFIG_LIB_EXPAT_VERSION`` and export ``expat.h`` to
other applications.

General entity expansion and DTD processing are disabled by default.  Enable
``CONFIG_LIB_EXPAT_GENERAL_ENTITIES`` or ``CONFIG_LIB_EXPAT_DTD`` only when an
application requires them and its XML input policy has been reviewed.

Expat uses the MIT license, so ``CONFIG_ALLOW_MIT_COMPONENTS`` must be enabled.
``CONFIG_S2OPC_XML_LOADERS`` uses this library for S2OPC XML configuration.
