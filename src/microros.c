#include "./config.h"
#include "./microros_usbcdc_transport.h"
#include "./microros_allocators.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>

#define CHECK_AND_CONTINUE(X) if (!ret || !X){ret = false;}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
	static volatile int64_t init = -1; \
	if (init == -1) { init = uxr_millis();} \
	if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

rcl_allocator_t allocator;
rclc_support_t support;
rcl_init_options_t init_options;

rcl_node_t node;
rclc_executor_t executor;

rcl_timer_t pub_timer;
rcl_publisher_t pub;
std_msgs__msg__Int32 msg = {};

void pub_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	rcl_publish(&pub, &msg, NULL);
	msg.data++;
}

bool init_microros_entites() {
	bool ret = true;

	allocator = rcl_get_default_allocator();

	init_options = rcl_get_zero_initialized_init_options();
	CHECK_AND_CONTINUE(RCL_RET_OK == rcl_init_options_init(&init_options, allocator));

	// create init_options
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  	(void)! rcl_init_options_fini(&init_options);

	// sync time
	CHECK_AND_CONTINUE(RCL_RET_OK == rmw_uros_sync_session(1000.0));

	// create node
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_node_init_default(&node, "tivac_node", "", &support));

	// create publishers
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_publisher_init_default(&pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "tivac_pub"));

	// create timer,
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_timer_init_default(&pub_timer, &support, RCL_MS_TO_NS(1000), pub_timer_callback));

	// create executor
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_executor_init(&executor, &support.context, 1, &allocator));
	CHECK_AND_CONTINUE(RCL_RET_OK == rclc_executor_add_timer(&executor, &pub_timer));

	return ret;
}

bool fini_microros_entities() {
	bool ret = true;

 	rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
	(void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

	(void)! rcl_publisher_fini(&pub, &node);
	(void)! rcl_timer_fini(&pub_timer);
	(void)! rclc_executor_fini(&executor);
	(void)! rcl_node_fini(&node);
	(void)! rclc_support_fini(&support);

	free_all_heap();

	return ret;
}

void microros_app(void * arg)
{
	allocator = rcutils_get_zero_initialized_allocator();
	allocator.allocate = custom_allocate;
	allocator.deallocate = custom_deallocate;
	allocator.reallocate = custom_reallocate;
	allocator.zero_allocate = custom_zero_allocate;

  	(void)! rcutils_set_default_allocator(&allocator);

    rmw_uros_set_custom_transport(
		true,
		(void *) NULL,
		tivac_usbcdc_transport_open,
		tivac_usbcdc_transport_close,
		tivac_usbcdc_transport_write,
		tivac_usbcdc_transport_read
	);

	enum states {
		WAITING_AGENT,
		AGENT_AVAILABLE,
		AGENT_CONNECTED,
		AGENT_DISCONNECTED
	} state = WAITING_AGENT;

	while(1) {
		switch (state) {
		case WAITING_AGENT:
			EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
			break;
		case AGENT_AVAILABLE:
			state = (true == init_microros_entites()) ? AGENT_CONNECTED : WAITING_AGENT;
			if (state == WAITING_AGENT) {
				fini_microros_entities();
			};
			break;
		case AGENT_CONNECTED:
			EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
			if (state == AGENT_CONNECTED) {
				rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
			}
			break;
		case AGENT_DISCONNECTED:
			fini_microros_entities();
			state = WAITING_AGENT;
			break;
		default:
			break;
    	}
	}
}