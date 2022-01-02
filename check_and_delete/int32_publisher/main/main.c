#include <stdio.h>
#include <unistd.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>

#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <driver/gpio.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

// Constants
#define SLEEP_TIME 10

// PINS
#define LED_BUILTIN 2

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

//rcl_publisher_t publisher;
rcl_publisher_t led_state_publisher;
rcl_subscription_t led_input_subscriber;

//std_msgs__msg__Int32 msg;
std_msgs__msg__Int32 LedStateMsg;
std_msgs__msg__Int32 LedInputMsg;

//td_msgs__msg__Int32 LedInputMsg;


void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		LedStateMsg.data = gpio_get_level(LED_BUILTIN);
		RCSOFTCHECK(rcl_publish(&led_state_publisher, &LedStateMsg, NULL));
		//LedStateMsg.data++;
	}
}

void subscription_callback(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
	/*global led_input_data = 0;
	led_input_data = msg->data;
	return led_input_data;*/
	gpio_set_level(LED_BUILTIN, msg->data);

    /*if((msg->data) == 0){
        gpio_set_level(LED_BUILTIN, 1);		
    }

    if((msg->data) == 1){
        gpio_set_level(LED_BUILTIN, 0);		
    }	*/
}

void blink() {

			
	
}

void setupPins() {

    // Led. Set it to GPIO_MODE_INPUT_OUTPUT, because we want to read back the state we set it to.
    gpio_reset_pin(LED_BUILTIN);
    gpio_set_direction(LED_BUILTIN, GPIO_MODE_INPUT_OUTPUT);
    //gpio_set_level(LED_BUILTIN, !gpio_get_level(LED_BUILTIN));    
}


void app_main(void * arg)
{
	setupPins();
	blink();
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "blink_on_input", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&led_state_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"led_state"));
	
	// create subscriber (...... THIS PART IS OBSTRUCTING PUBLISHING.......)
	RCCHECK(rclc_subscription_init_default(
		&led_input_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"led_input"));

	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &led_input_subscriber,&LedInputMsg,
		&subscription_callback, ON_NEW_DATA));


	

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(100000);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&led_state_publisher, &node));
	RCCHECK(rcl_subscription_fini(&led_input_subscriber, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}
