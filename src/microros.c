#include "./config.h"
#include "./microros_usbcdc_transport.h"
#include "./microros_allocators.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);while(1);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

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
	RCSOFTCHECK(rcl_publish(&pub, &msg, NULL));
	msg.data++;
}

void micro_ros_task(void * arg)
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

	allocator = rcl_get_default_allocator();

	init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// create node
	RCCHECK(rclc_node_init_default(&node, "tivac_node", "", &support));

	// create publishers
	RCCHECK(rclc_publisher_init_best_effort(&pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "tivac_pub"));

	// create timer,
	RCCHECK(rclc_timer_init_default(&pub_timer, &support, RCL_MS_TO_NS(20), pub_timer_callback));

	// create executor
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &pub_timer));

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
	}
}