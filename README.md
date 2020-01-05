**linnvme** is a kernel device driver for FreeBSD that provides compatibility with Linux NVMe IOCTL commands.

The functionality provided by the Linux and FreeBSD NVMe drivers is similar but differs in the interface. This driver primarily translates between the FreeBSD and Linux passthru structures.

One notable difference between Linux and FreeBSD concerns the device nodes used for NVMe namespaces. Linux expects a name format of `nvmeXXnYY` and expects the node to be a block device. FreeBSD, on the other hand, uses a name format of `nvmeXXnsYY` and exposes the node as a character device. To accommodate Linux, the `linnvme` driver creates new device nodes for each namespace conforming to the Linux requirements.

#### Build
Run `make` on a FreeBSD 12 or -current system.

#### Requirements
The driver extends the Linux compatibility and requires the `linux_common` and `linux64` drivers. But running `kldload linnvme.ko` should automatically load the required drivers.

Linux applications trying to open the NVMe devices will look under `/compat/linux/dev/`. To add the device file system for Linux applications, run:

```# mount -t devfs devfs /compat/linux/dev/```
