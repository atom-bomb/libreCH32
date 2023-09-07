# libreCH32 #

* libreCH32 is a more or less portable command line interface for programming CH32F103 chips. It replaces the Windows-only WCHISPTOOL provided by the manufacturer.

* Only CH32F103 is supported.

### How to build ###

Just type `make`.

The resultant binary is written to a subdirectory corresponding to the target architecture (ie, x86\_64/bin/libreCH32).

### How to run ###

to program firmware, verify and then reboot the device:

```
libreCH32 -p /dev/ttyUSB0 -W path_to/firmware.bin -r
```

to see more options, get help:

```
libreCH32 -h
```
