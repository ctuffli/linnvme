SRCS=linux_nvme.c
KMOD=linnvme

SRCS+=	bus_if.h \
	device_if.h

.include <bsd.kmod.mk>
