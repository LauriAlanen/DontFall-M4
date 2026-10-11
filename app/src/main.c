/*
 * DontFall M4 firmware: OpenAMP remote on the STM32MP157 Cortex-M4.
 *
 * Announces an "rpmsg-tty" channel (Linux: /dev/ttyRPMSG0) and answers every
 * message with "hello from M4: <message>".
 *
 * Derived from zephyr/samples/subsys/ipc/openamp_rsc_table
 * Copyright (c) 2020, STMICROELECTRONICS
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/ipm.h>
#include <zephyr/sys/printk.h>
#include <string.h>

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

#define REPLY_PREFIX     "hello from M4: "
#define REPLY_PREFIX_LEN (sizeof(REPLY_PREFIX) - 1)
/* rpmsg payload limit with the default 512 byte buffers */
#define RPMSG_MAX_PAYLOAD 496

#define MNG_STACK_SIZE 1024
/* extra room for the tx buffer */
#define TTY_STACK_SIZE (1024 + RPMSG_MAX_PAYLOAD)

K_THREAD_STACK_DEFINE(thread_mng_stack, MNG_STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_tty_stack, TTY_STACK_SIZE);
static struct k_thread thread_mng_data;
static struct k_thread thread_tty_data;

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

struct rpmsg_rcv_msg {
	void *data;
	size_t len;
};

static struct rpmsg_endpoint tty_ept;
static struct rpmsg_rcv_msg tty_msg;

static K_SEM_DEFINE(data_sem, 0, 1);
static K_SEM_DEFINE(data_tty_sem, 0, 1);

static void platform_ipm_callback(const struct device *dev, void *context,
				  uint32_t id, volatile void *data)
{
	k_sem_give(&data_sem);
}

static int rpmsg_recv_tty_callback(struct rpmsg_endpoint *ept, void *data,
				   size_t len, uint32_t src, void *priv)
{
	struct rpmsg_rcv_msg *msg = priv;

	rpmsg_hold_rx_buffer(ept, data);
	msg->data = data;
	msg->len = len;
	k_sem_give(&data_tty_sem);

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

static void app_rpmsg_tty(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	unsigned char tx_buff[RPMSG_MAX_PAYLOAD];
	int ret;

	/* released by rpmsg_mng_task once the rpmsg device exists */
	k_sem_take(&data_tty_sem, K_FOREVER);

	tty_ept.priv = &tty_msg;
	ret = rpmsg_create_ept(&tty_ept, rpdev, "rpmsg-tty", RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
			       rpmsg_recv_tty_callback, NULL);
	if (ret) {
		printk("rpmsg-tty endpoint failed: %d\n", ret);
		return;
	}
	printk("rpmsg-tty ready\n");

	while (tty_ept.addr != RPMSG_ADDR_ANY) {
		k_sem_take(&data_tty_sem, K_FOREVER);
		if (tty_msg.len) {
			size_t len = MIN(tty_msg.len, sizeof(tx_buff) - REPLY_PREFIX_LEN);

			memcpy(tx_buff, REPLY_PREFIX, REPLY_PREFIX_LEN);
			memcpy(&tx_buff[REPLY_PREFIX_LEN], tty_msg.data, len);
			rpmsg_send(&tty_ept, tx_buff, REPLY_PREFIX_LEN + len);
			rpmsg_release_rx_buffer(&tty_ept, tty_msg.data);
		}
		tty_msg.len = 0;
		tty_msg.data = NULL;
	}
	rpmsg_destroy_ept(&tty_ept);
}

static void rpmsg_mng_task(void *arg1, void *arg2, void *arg3)
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

	k_sem_give(&data_tty_sem);

	while (1) {
		if (k_sem_take(&data_sem, K_FOREVER) == 0) {
			rproc_virtio_notified(rvdev.vdev, VRING1_ID);
		}
	}

task_end:
	ipm_set_enabled(ipm_handle, 0);
	rpmsg_deinit_vdev(&rvdev);
	metal_finish();
	printk("OpenAMP setup failed\n");
}

int main(void)
{
	printk("dontfall-m4 up\n");

	k_thread_create(&thread_mng_data, thread_mng_stack, MNG_STACK_SIZE, rpmsg_mng_task,
			NULL, NULL, NULL, K_PRIO_COOP(8), 0, K_NO_WAIT);
	k_thread_create(&thread_tty_data, thread_tty_stack, TTY_STACK_SIZE, app_rpmsg_tty,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);
	return 0;
}
