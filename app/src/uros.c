/*
 * micro-ROS node on the M4. See uros.h.
 *
 * Runs in its own preemptible thread, below the coop ipc threads: rclc's
 * executor blocks in the transport read while it waits for data.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "uros.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int32.h>

#include "uros_transport.h"

#define UROS_STACK_SIZE   16384
#define UROS_PRIORITY     K_PRIO_PREEMPT(5)
#define PUBLISH_PERIOD_MS 1000
#define AGENT_PING_MS     100

/* print the failing call and leave the thread */
#define UROS_CHECK(fn)                                                         \
	do {                                                                   \
		rcl_ret_t rc_ = (fn);                                          \
		if (rc_ != RCL_RET_OK) {                                       \
			printk("uros: %s failed: %d\n", #fn, (int)rc_);        \
			return;                                                \
		}                                                              \
	} while (0)

K_THREAD_STACK_DEFINE(uros_stack, UROS_STACK_SIZE);
static struct k_thread uros_thread;

static rcl_publisher_t publisher;
static std_msgs__msg__Int32 msg;

static void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
	ARG_UNUSED(last_call_time);

	if (!timer) {
		return;
	}

	rcl_ret_t rc = rcl_publish(&publisher, &msg, NULL);

	if (rc != RCL_RET_OK) {
		printk("uros: publish failed: %d\n", (int)rc);
	}
	msg.data++;
}

static void uros_task(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;
	rcl_node_t node;
	rcl_timer_t timer;
	rclc_executor_t executor;

	if (uros_transport_init()) {
		printk("uros: transport init failed\n");
		return;
	}

	printk("uros: waiting for the micro-ROS Agent\n");
	while (rmw_uros_ping_agent(AGENT_PING_MS, 1) != RMW_RET_OK) {
		k_sleep(K_MSEC(500));
	}
	printk("uros: Agent found\n");

	UROS_CHECK(rclc_support_init(&support, 0, NULL, &allocator));
	UROS_CHECK(rclc_node_init_default(&node, "dontfall_m4", "", &support));
	UROS_CHECK(rclc_publisher_init_default(&publisher, &node,
					       ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
					       "~/counter"));
	UROS_CHECK(rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(PUBLISH_PERIOD_MS),
					    timer_callback, true));
	UROS_CHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	UROS_CHECK(rclc_executor_add_timer(&executor, &timer));
	printk("uros: publishing /dontfall_m4/counter\n");

	for (;;) {
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
	}
}

int uros_start(void)
{
	k_thread_create(&uros_thread, uros_stack, K_THREAD_STACK_SIZEOF(uros_stack), uros_task,
			NULL, NULL, NULL, UROS_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&uros_thread, "uros");
	return 0;
}
