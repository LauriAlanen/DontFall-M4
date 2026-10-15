/*
 * micro-ROS node on the M4. See uros.h.
 *
 * Runs in its own preemptible thread, below the coop ipc threads: rclc's
 * executor blocks in the transport read while it waits for data.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "uros.h"

#include <string.h>
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
/* how often to look for the Agent while it's away */
#define AGENT_SEARCH_MS   500
/* how often to check a connected Agent is still there */
#define AGENT_CHECK_MS    1000

K_THREAD_STACK_DEFINE(uros_stack, UROS_STACK_SIZE);
static struct k_thread uros_thread;

static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rcl_publisher_t publisher;
static rcl_timer_t timer;
static rclc_executor_t executor;
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

/* print the failing call and bail out of create_entities() */
#define UROS_CHECK(fn)                                                         \
	do {                                                                   \
		rcl_ret_t rc_ = (fn);                                          \
		if (rc_ != RCL_RET_OK) {                                       \
			printk("uros: %s failed: %d\n", #fn, (int)rc_);       \
			return false;                                          \
		}                                                              \
	} while (0)

/* Open a session with the Agent and create the node and its entities. */
static bool create_entities(void)
{
	UROS_CHECK(rclc_support_init(&support, 0, NULL, &allocator));
	UROS_CHECK(rclc_node_init_default(&node, "dontfall_m4", "", &support));
	UROS_CHECK(rclc_publisher_init_default(&publisher, &node,
					       ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
					       "~/counter"));
	UROS_CHECK(rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(PUBLISH_PERIOD_MS),
					    timer_callback, true));
	UROS_CHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	UROS_CHECK(rclc_executor_add_timer(&executor, &timer));
	return true;
}

/*
 * Free everything create_entities() made. The Agent is gone (or never
 * answered), so don't wait for it to confirm the deletes. Also safe after
 * a partial create: the fini calls skip zero-initialized entities.
 */
static void destroy_entities(void)
{
	rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);

	if (rmw_context) {
		(void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
	}

	/* errors are expected here (entities never made, no Agent to answer) */
	rcl_ret_t rc;

	rc = rclc_executor_fini(&executor);
	rc = rcl_timer_fini(&timer);
	rc = rcl_publisher_fini(&publisher, &node);
	rc = rcl_node_fini(&node);
	rc = rclc_support_fini(&support);
	(void)rc;

	executor = rclc_executor_get_zero_initialized_executor();
	timer = rcl_get_zero_initialized_timer();
	publisher = rcl_get_zero_initialized_publisher();
	node = rcl_get_zero_initialized_node();
	memset(&support, 0, sizeof(support));
	/* fini on half-made entities sets errors nobody reads */
	rcl_reset_error();
}

static bool agent_alive(void)
{
	return rmw_uros_ping_agent(AGENT_PING_MS, 1) == RMW_RET_OK;
}

static void uros_task(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	allocator = rcl_get_default_allocator();
	executor = rclc_executor_get_zero_initialized_executor();
	timer = rcl_get_zero_initialized_timer();
	publisher = rcl_get_zero_initialized_publisher();
	node = rcl_get_zero_initialized_node();

	if (uros_transport_init()) {
		printk("uros: transport init failed\n");
		return;
	}

	/*
	 * Find the Agent, create everything, spin until a ping goes
	 * unanswered, then tear down and start over. A restarted Agent has
	 * forgotten our session, so everything has to be created again.
	 */
	for (;;) {
		printk("uros: waiting for the micro-ROS Agent\n");
		while (!agent_alive()) {
			k_sleep(K_MSEC(AGENT_SEARCH_MS));
		}
		printk("uros: Agent found\n");

		if (!create_entities()) {
			destroy_entities();
			k_sleep(K_MSEC(AGENT_SEARCH_MS));
			continue;
		}
		printk("uros: publishing /dontfall_m4/counter\n");

		int64_t next_check = k_uptime_get() + AGENT_CHECK_MS;

		for (;;) {
			rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
			if (k_uptime_get() < next_check) {
				continue;
			}
			next_check = k_uptime_get() + AGENT_CHECK_MS;
			if (!agent_alive()) {
				break;
			}
		}

		printk("uros: Agent lost\n");
		destroy_entities();
		uros_transport_reset();
	}
}

int uros_start(void)
{
	k_thread_create(&uros_thread, uros_stack, K_THREAD_STACK_SIZEOF(uros_stack), uros_task,
			NULL, NULL, NULL, UROS_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&uros_thread, "uros");
	return 0;
}
