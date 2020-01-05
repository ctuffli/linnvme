/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2020 Chuck Tuffli
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/capsicum.h>
#include <sys/conf.h>
#include <sys/module.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/proc.h>
#include <sys/stat.h>

#ifdef COMPAT_LINUX32
#include <machine/../linux32/linux.h>
#include <machine/../linux32/linux32_proto.h>
#else
#include <machine/../linux/linux.h>
#include <machine/../linux/linux_proto.h>
#endif

#include <compat/linux/linux_ioctl.h>

#include <dev/nvme/nvme.h>
#include <dev/nvme/nvme_private.h>

static d_ioctl_t linnvme_ioctl;

static struct cdevsw linnvme_cdevsw = {
	.d_version =	D_VERSION,
	.d_flags =	D_TRACKCLOSE,
	.d_name =	"linnvme",
	.d_ioctl =	linnvme_ioctl,
};

struct linnvme_ns_dev {
	struct cdev	*cdev;
	STAILQ_ENTRY(linnvme_ns_dev)	link;
};

static STAILQ_HEAD(, linnvme_ns_dev) ns_list = STAILQ_HEAD_INITIALIZER(ns_list);

static struct mtx linnvme_mtx;
MTX_SYSINIT(linnvme_mtx_init, &linnvme_mtx, "Linux NVMe lock", MTX_DEF);

MALLOC_DEFINE(M_LINNVME, "linnvme", "Memory used for Linux NVMe device");

/*
 * Linux NVMe IOCTL structures and definitions
 */

struct nvme_passthru_cmd {
	uint8_t		opcode;
	uint8_t		flags;
	uint16_t	rsvd;
	uint32_t	nsid;
	uint32_t	cdw2;
	uint32_t	cdw3;
	uint64_t	metadata;
	uint64_t	addr;
	uint32_t	metadata_len;
	uint32_t	data_len;
	uint32_t	cdw10;
	uint32_t	cdw11;
	uint32_t	cdw12;
	uint32_t	cdw13;
	uint32_t	cdw14;
	uint32_t	cdw15;
	uint32_t	timeout_ms;
	uint32_t	result;
};

#define LINUX_NVME_IOCTL(g, c)	((((uint8_t)(g)) << 8) | ((uint8_t)(c)))

#define LINUX_NVME_IOCTL_ID		LINUX_NVME_IOCTL('N', 0x40)
#define LINUX_NVME_IOCTL_ADMIN_CMD	LINUX_NVME_IOCTL('N', 0x41)
#define LINUX_NVME_IOCTL_SUBMIT_IO	LINUX_NVME_IOCTL('N', 0x42)
#define LINUX_NVME_IOCTL_IO_CMD		LINUX_NVME_IOCTL('N', 0x43)
#define LINUX_NVME_IOCTL_RESET		LINUX_NVME_IOCTL('N', 0x44)
#define LINUX_NVME_IOCTL_SUBSYS_RESET	LINUX_NVME_IOCTL('N', 0x45)
#define LINUX_NVME_IOCTL_RESCAN		LINUX_NVME_IOCTL('N', 0x46)

#define LINUX_NVME_IOCTL_MIN	LINUX_NVME_IOCTL_ID
#define LINUX_NVME_IOCTL_MAX	LINUX_NVME_IOCTL_RESCAN

static linux_ioctl_function_t linnvme_linux_ioctl;
static struct linux_ioctl_handler linnvme_ioctl_handler = {
	linnvme_linux_ioctl,
	LINUX_NVME_IOCTL_MIN,
	LINUX_NVME_IOCTL_MAX,
};

SYSINIT  (linnvme_linux_register,   SI_SUB_KLD, SI_ORDER_MIDDLE,
	  linux_ioctl_register_handler, &linnvme_ioctl_handler);
SYSUNINIT(linnvme_linux_unregister, SI_SUB_KLD, SI_ORDER_MIDDLE,
	  linux_ioctl_unregister_handler, &linnvme_ioctl_handler);

struct linux_nvme_pt_command {
	struct nvme_pt_command pt;
	int ioctl_cmd;
};

/*
 * Convert from a Linux passthru command to the native format
 */
static int
linnvme_cmd_convert(struct nvme_pt_command *npt, struct nvme_passthru_cmd *lpt)
{

	if (!npt || !lpt)
		return (EINVAL);

	npt->cmd.opc = lpt->opcode;
	npt->cmd.nsid = htole32(lpt->nsid);

	npt->cmd.cdw10 = htole32(lpt->cdw10);
	npt->cmd.cdw11 = htole32(lpt->cdw11);
	npt->cmd.cdw12 = htole32(lpt->cdw12);
	npt->cmd.cdw13 = htole32(lpt->cdw13);
	npt->cmd.cdw14 = htole32(lpt->cdw14);
	npt->cmd.cdw15 = htole32(lpt->cdw15);

	npt->buf = (void *)lpt->addr;
	npt->len = lpt->data_len;

	npt->is_read = (lpt->opcode & 0x2) != 0;

	return (0);
}

/*
 * Convert the native passthru completion to the Linux format
 */
static int
linnvme_cpl_convert(struct nvme_pt_command *npt, struct nvme_passthru_cmd *lpt)
{

	lpt->result = le32toh(npt->cpl.cdw0);
	return (le16toh(npt->cpl.status));
}

/*
 * Main Linux NVMe IOCTL handler
 */
static int
linnvme_linux_ioctl(struct thread *td, struct linux_ioctl_args *args)
{
	struct linux_nvme_pt_command lpt = { { { 0 }, }, };
	struct nvme_get_nsid gnsid = { { 0 } };
	cap_rights_t rights;
	caddr_t arg = NULL;
	u_long cmd;
	struct file *fp;
	int err = 0;

	err = fget(td, args->fd, cap_rights_init(&rights, CAP_IOCTL), &fp);
	if (err != 0)
		return (err);
	cmd = args->cmd & UINT16_MAX;

	lpt.ioctl_cmd = cmd;

	switch (cmd) {
	case LINUX_NVME_IOCTL_ID:
		err = fo_ioctl(fp, NVME_GET_NSID, &gnsid, td->td_ucred, td);
		break;
	case LINUX_NVME_IOCTL_ADMIN_CMD:
		err = linnvme_cmd_convert(&lpt.pt, (struct nvme_passthru_cmd *)args->arg);
		if (err)
			break;
		err = fo_ioctl(fp, NVME_PASSTHROUGH_CMD, &lpt, td->td_ucred, td);
		break;
	case LINUX_NVME_IOCTL_SUBMIT_IO:
	case LINUX_NVME_IOCTL_IO_CMD:
		printf("NVMe I/O command passthru not supported\n");
		return (EINVAL);
		break;
	case LINUX_NVME_IOCTL_RESET:
		err = fo_ioctl(fp, NVME_RESET_CONTROLLER, arg, td->td_ucred, td);
		break;
	case LINUX_NVME_IOCTL_SUBSYS_RESET:
		printf("got LINUX_NVME_IOCTL_SUBSYS_RESET\n");
		return (EOPNOTSUPP);
		break;
	case LINUX_NVME_IOCTL_RESCAN:
		printf("got LINUX_NVME_IOCTL_RESCAN\n");
		return (EOPNOTSUPP);
		break;
	default:
		printf("unknown command %#lx\n", cmd);
		return (EINVAL);
	}

	if (err < 0)
		return (err);

	switch (cmd) {
	case LINUX_NVME_IOCTL_ADMIN_CMD:
		err = linnvme_cpl_convert(&lpt.pt, (struct nvme_passthru_cmd *)args->arg);
		break;
	case LINUX_NVME_IOCTL_ID:
		td->td_retval[0] = gnsid.nsid;
		break;
	}

	return (err);
}

/*
 * IOCTL handler for Linux NVMe namespace device nodes
 */
static int
linnvme_ioctl(struct cdev *cdev, u_long cmd, caddr_t arg, int flag, struct thread *td)
{
	int err = EINVAL;

	switch (cmd) {
	case NVME_PASSTHROUGH_CMD:
	{
		struct linux_nvme_pt_command *lpt;
		uint32_t nsid;

		lpt = (struct linux_nvme_pt_command *)arg;

		nsid = le32toh(lpt->pt.cmd.nsid);
		/*
		 * Linux clients will issue Admin commands to namespace devices
		 */
		err = nvme_ctrlr_passthrough_cmd(cdev->si_drv1,
		    &lpt->pt, nsid,
		    1 /* is_user_buffer */,
		    lpt->ioctl_cmd == LINUX_NVME_IOCTL_ADMIN_CMD /* is_admin_cmd */);
		break;
	}
	case NVME_GET_NSID:
		err = nvme_ns_ioctl_process(cdev->si_drv2, cmd, arg, flag, td);
		break;
	default:
		printf("%s: unsupported IOCTL '%c' %#x\n", __func__,
		    (char)(cmd >> 8), (uint8_t)(cmd & 0xff));
	}

	return (err);
}

/*
 * Create a Linux compatible namespace device node
 *
 * Linux expects namespace device nodes to be block devices (i.e. S_IFBLK)
 * and to use the naming convention nvmeXXnYY where XX is the unit number
 * and YY is the namespace ID.
 */
static struct cdev *
linnvme_dev_add(const char *name, int nsid)
{
	struct make_dev_args args;
	struct cdev *linnvme_dev = NULL;
	int error;

	make_dev_args_init(&args);

	args.mda_devsw = &linnvme_cdevsw;
	args.mda_uid = UID_ROOT;
	args.mda_gid = GID_WHEEL;
	args.mda_mode = 0600 | S_IFBLK;

	error = make_dev_s(&args,
	    &linnvme_dev,
	    "%sn%d", name, nsid);

	return (linnvme_dev);
}

/*
 * Search for devices in the 'nvme' device class and create a Linux
 * compatible device for each namespace
 */
static int
linnvme_find_devs(void)
{
	devclass_t dc;
	struct nvme_controller *ctrlr;
	device_t *devlist;
	int dcount = 0;

	dc = devclass_find("nvme");
	if (dc == NULL) {
		printf("no NVMe device class\n");
		return (ENODEV);
	}

	if (devclass_get_devices(dc, &devlist, &dcount)) {
		printf("no NVMe devices found\n");
		return (ENODEV);
	}

	for (int i = 0; i < dcount; i++) {
		ctrlr = device_get_softc(devlist[i]);
		if (ctrlr) {
			struct linnvme_ns_dev *ns_dev;
			const char *nameunit;
			int nsid;
			
			nameunit = device_get_nameunit(devlist[i]);
			for (nsid = 0; nsid < NVME_MAX_NAMESPACES; nsid++) {
				if (ctrlr->ns[nsid].cdev == NULL)
					continue;

				ns_dev = malloc(sizeof(*ns_dev), M_LINNVME, M_ZERO | M_WAITOK);
				if (ns_dev == NULL)
					continue;
				
				ns_dev->cdev = linnvme_dev_add(nameunit, ctrlr->ns[nsid].id);
				ns_dev->cdev->si_drv1 = ctrlr;
				ns_dev->cdev->si_drv2 = &ctrlr->ns[nsid];

				mtx_lock(&linnvme_mtx);
				STAILQ_INSERT_TAIL(&ns_list, ns_dev, link);
				mtx_unlock(&linnvme_mtx);
			}
		}
	}

	free(devlist, M_TEMP);

	return (0);
}

static void
linnvme_destroy_devs(void)
{
	struct linnvme_ns_dev *ns_dev;

	while (!STAILQ_EMPTY(&ns_list)) {
		mtx_lock(&linnvme_mtx);
		ns_dev = STAILQ_FIRST(&ns_list);
		STAILQ_REMOVE_HEAD(&ns_list, link);
		mtx_unlock(&linnvme_mtx);

		destroy_dev(ns_dev->cdev);
		free(ns_dev, M_LINNVME);
	}
}

static int
linnvme_loader(struct module *m, int what, void *arg)
{
	int error = 0;

	switch (what) {
	case MOD_LOAD:
		linnvme_find_devs();
		break;
	case MOD_UNLOAD:
		linnvme_destroy_devs();
		break;
	default:
		error = EOPNOTSUPP;
	}

	return (error);
}

static moduledata_t linnvme_mod = {
	"linnvme",
	linnvme_loader,
	NULL
};

MODULE_DEPEND(linnvme, linux_common, 1, 1, 1);
MODULE_DEPEND(linnvme, linux64, 1, 1, 1);
DECLARE_MODULE(linnvme, linnvme_mod, SI_SUB_KLD, SI_ORDER_ANY);
