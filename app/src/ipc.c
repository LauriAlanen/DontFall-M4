/*
 * OpenAMP remote side of the A7 <-> M4 link: libmetal/virtio setup on the
 * resource table, the IPCC mailbox kick, and the "rpmsg-tty" endpoint.
 *
 * Two threads:
 *  - mng: brings the rpmsg device up, then turns every mailbox interrupt
 *    into rproc_virtio_notified(), which runs the endpoint rx callback.
 *  - rx:  creates the endpoint and hands each received message to the
 *    user callback, outside the virtio notification path.
 *
 * Derived from zephyr/samples/subsys/ipc/openamp_rsc_table
 * Copyright (c) 2020, STMICROELECTRONICS
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ipc.h"

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/ipm.h>
#include <zephyr/sys/printk.h>

#include <openamp/open_amp.h>
#include <metal/sys.h>
#include <metal/io.h>
#include <resource_table.h>

#if !DT_HAS_CHOSEN(zephyr_ipc_shm)
#error "Requires definition of shared memory for rpmsg"
#endif

#define SHM_NODE       DT_CHOSEN(zephyr_ipc_shm)
#define SHM_START_ADDR DT_REG_ADDR(SHM_NODE)
#define SHM_SIZE       DT_REG_SIZE(SHM_NODE)

#define MNG_STACK_SIZE 1024
#define RX_STACK_SIZE  1024

K_THREAD_STACK_DEFINE(thread_mng_stack, MNG_STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_rx_stack, RX_STACK_SIZE);
static struct k_thread thread_mng_data;
static struct k_thread thread_rx_data;

static const struct device *const ipm_handle = DEVICE_DT_GET(DT_CHOSEN(zephyr_ipc));

static metal_phys_addr_t shm_physmap = SHM_START_ADDR;
static metal_phys_addr_t rsc_tab_physmap;
static struct metal_io_region shm_io_data;
static struct metal_io_region rsc_io_data;
static struct metal_io_region *shm_io = &shm_io_data;
static struct metal_io_region *rsc_io = &rsc_io_data;

static struct rpmsg_virtio_device rvdev;
static struct fw_resource_table *rsc_table;
static struct rpmsg_device *rpdev;

static struct rpmsg_endpoint tty_ept;
static ipc_rx_cb_t user_rx_cb;

struct rx_msg {
	void *data;
	size_t len;
};

/*
 * One slot per rpmsg buffer: a single rproc_virtio_notified() call can
 * deliver several messages before the rx thread gets to run.
 */
K_MSGQ_DEFINE(rx_queue, sizeof(struct rx_msg), CONFIG_OPENAMP_RSC_TABLE_NUM_RPMSG_BUFF, 4);

static K_SEM_DEFINE(mbox_sem, 0, 1);
static K_SEM_DEFINE(rpdev_ready_sem, 0, 1);

static void platform_ipm_callback(const struct device *dev, void *context,
				  uint32_t id, volatile void *data)
{
	k_sem_give(&mbox_sem);
}

static int rpmsg_recv_tty_callback(struct rpmsg_endpoint *ept, void *data,
				   size_t len, uint32_t src, void *priv)
{
	struct rx_msg msg = {.data = data, .len = len};

	/* keep the buffer until the rx thread has consumed it */
	rpmsg_hold_rx_buffer(ept, data);
	if (k_msgq_put(&rx_queue, &msg, K_NO_WAIT)) {
		rpmsg_release_rx_buffer(ept, data);
		printk("ipc: rx queue full, message dropped\n");
	}

	return RPMSG_SUCCESS;
}

static void new_service_cb(struct rpmsg_device *rdev, const char *name, uint32_t src)
{
	printk("unexpected ns service %s\n", name);
}

static int mailbox_notify(void *priv, uint32_t id)
{
	ARG_UNUSED(priv);
	ipm_send(ipm_handle, 0, id, NULL, 0);
	return 0;
}

static int platform_init(void)
{
	struct metal_init_params metal_params = METAL_INIT_DEFAULTS;
	int rsc_size;
	int status;

	status = metal_init(&metal_params);
	if (status) {
		printk("metal_init failed: %d\n", status);
		return -1;
	}

	metal_io_init(shm_io, (void *)SHM_START_ADDR, &shm_physmap, SHM_SIZE, -1, 0, NULL);

	rsc_table_get(&rsc_table, &rsc_size);
	rsc_tab_physmap = (uintptr_t)rsc_table;
	metal_io_init(rsc_io, rsc_table, &rsc_tab_physmap, rsc_size, -1, 0, NULL);

	if (!device_is_ready(ipm_handle)) {
		printk("IPM device not ready\n");
		return -1;
	}
	ipm_register_callback(ipm_handle, platform_ipm_callback, NULL);

	status = ipm_set_enabled(ipm_handle, 1);
	if (status) {
		printk("ipm_set_enabled failed\n");
		return -1;
	}

	return 0;
}

static struct rpmsg_device *platform_create_rpmsg_vdev(void)
{
	struct fw_rsc_vdev_vring *vring_rsc;
	struct virtio_device *vdev;
	int ret;

	vdev = rproc_virtio_create_vdev(VIRTIO_DEV_DEVICE, VDEV_ID,
					rsc_table_to_vdev(rsc_table),
					rsc_io, NULL, mailbox_notify, NULL);
	if (!vdev) {
		printk("failed to create vdev\n");
		return NULL;
	}

	/* wait for the Linux side to finish rpmsg init */
	rproc_virtio_wait_remote_ready(vdev);

	vring_rsc = rsc_table_get_vring0(rsc_table);
	ret = rproc_virtio_init_vring(vdev, 0, vring_rsc->notifyid, (void *)vring_rsc->da,
				      rsc_io, vring_rsc->num, vring_rsc->align);
	if (ret) {
		goto failed;
	}

	vring_rsc = rsc_table_get_vring1(rsc_table);
	ret = rproc_virtio_init_vring(vdev, 1, vring_rsc->notifyid, (void *)vring_rsc->da,
				      rsc_io, vring_rsc->num, vring_rsc->align);
	if (ret) {
		goto failed;
	}

	ret = rpmsg_init_vdev(&rvdev, vdev, new_service_cb, shm_io, NULL);
	if (ret) {
		goto failed;
	}

	return rpmsg_virtio_get_rpmsg_device(&rvdev);

failed:
	printk("rpmsg vdev setup failed: %d\n", ret);
	rproc_virtio_remove_vdev(vdev);
	return NULL;
}

static void ipc_rx_task(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	struct rx_msg msg;
	int ret;

	/* released by ipc_mng_task once the rpmsg device exists */
	k_sem_take(&rpdev_ready_sem, K_FOREVER);

	ret = rpmsg_create_ept(&tty_ept, rpdev, "rpmsg-tty", RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
			       rpmsg_recv_tty_callback, NULL);
	if (ret) {
		printk("rpmsg-tty endpoint failed: %d\n", ret);
		return;
	}
	printk("rpmsg-tty ready\n");

	while (tty_ept.addr != RPMSG_ADDR_ANY) {
		k_msgq_get(&rx_queue, &msg, K_FOREVER);
		if (msg.len) {
			user_rx_cb(msg.data, msg.len);
		}
		rpmsg_release_rx_buffer(&tty_ept, msg.data);
	}
	rpmsg_destroy_ept(&tty_ept);
}

static void ipc_mng_task(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	if (platform_init()) {
		goto task_end;
	}

	rpdev = platform_create_rpmsg_vdev();
	if (!rpdev) {
		goto task_end;
	}

	k_sem_give(&rpdev_ready_sem);

	while (1) {
		if (k_sem_take(&mbox_sem, K_FOREVER) == 0) {
			rproc_virtio_notified(rvdev.vdev, VRING1_ID);
		}
	}

task_end:
	ipm_set_enabled(ipm_handle, 0);
	rpmsg_deinit_vdev(&rvdev);
	metal_finish();
	printk("OpenAMP setup failed\n");
}

int ipc_start(ipc_rx_cb_t rx_cb)
{
	if (!rx_cb) {
		return -EINVAL;
	}
	user_rx_cb = rx_cb;

	k_thread_create(&thread_mng_data, thread_mng_stack, MNG_STACK_SIZE, ipc_mng_task,
			NULL, NULL, NULL, K_PRIO_COOP(8), 0, K_NO_WAIT);
	k_thread_create(&thread_rx_data, thread_rx_stack, RX_STACK_SIZE, ipc_rx_task,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);
	return 0;
}

int ipc_send(const void *data, size_t len)
{
	return rpmsg_send(&tty_ept, data, len);
}
