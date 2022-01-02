/* WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "mqtt_client.h"

#include "hc-sr04_driver.h"
#include "hc-sr04_driver.c"

#include <unistd.h>
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include "uxr/client/config.h"
#include <driver/gpio.h>

#include <inttypes.h>
#include <math.h>
#include <geometry_msgs/msg/twist.h>
#include <driver/ledc.h>


//static const char *TAG = "main_app";

// Macro functions
#define constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}


// Constants
#define FRAME_TIME 100 // 1000 / FRAME_TIME = FPS
#define SLEEP_TIME 10

// PINS
/*#define LED_BUILTIN 33
#define PIN_LEFT_FORWARD 12
#define PIN_LEFT_BACKWARD 13
#define PIN_RIGHT_FORWARD 15
#define PIN_RIGHT_BACKWARD 14

// PWM Channels (Reserve channel 0 and 1 for camera)
#define PWM_LEFT_FORWARD LEDC_CHANNEL_2
#define PWM_LEFT_BACKWARD LEDC_CHANNEL_3
#define PWM_RIGHT_FORWARD LEDC_CHANNEL_4
#define PWM_RIGHT_BACKWARD LEDC_CHANNEL_5

// Other PWM settings
#define PWM_FREQUENCY 50
#define PWM_RESOLUTION LEDC_TIMER_12_BIT
#define PWM_TIMER LEDC_TIMER_1
#define PWM_MODE LEDC_HIGH_SPEED_MODE*/

// These values are determined by experiment and are unique to every robot
#define PWM_MOTOR_MIN 750    // The value where the motor starts moving
#define PWM_MOTOR_MAX 4095   // Full speed (2^12 - 1)

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif


//rcl_publisher_t publisher;
rcl_publisher_t led_state_publisher;
rcl_subscription_t led_input_subscriber;

//rcl_publisher_t publisher;
//std_msgs__msg__Int32 msg;


std_msgs__msg__Int32 LedStateMsg;
std_msgs__msg__Int32 LedInputMsg;
geometry_msgs__msg__Twist msg;

// Function forward declarations
/*void setupPins();
void setupRos();
void cmd_vel_callback(const void *msgin);*/
//void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
float fmap(float val, float in_min, float in_max, float out_min, float out_max);


int hcsr_task()
{
    
    float current_distance = 0.0;

    //double CALIB_DISTANCE = 0.0;

    hcsr_setup_pins();

    // Caliberate the sensor first
    //CALIB_DISTANCE = hcsr_caliberate_sensor();
    //ESP_LOGI(TAG,"going to get distance"); 
    
    current_distance = (int)hcsr_get_distance_in();
    //printf("distance is: %d", current_distance);
    ESP_LOGI(TAG,"current Distance: %f",current_distance);    
    
    return current_distance;
	}


/*void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		LedStateMsg.data = hcsr_task();
		RCSOFTCHECK(rcl_publish(&led_state_publisher, &LedStateMsg, NULL));
		printf("send ok\n");
		//LedStateMsg.data++;
	}
}*/

void subscription_callback(const void * msgin){
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;	
	gpio_set_level(LED_BUILTIN, msg->data);
	
}

/*void setupPins() {

    // Led. Set it to GPIO_MODE_INPUT_OUTPUT, because we want to read back the state we set it to.
    gpio_reset_pin(LED_BUILTIN);
    gpio_set_direction(LED_BUILTIN, GPIO_MODE_INPUT_OUTPUT);
    //gpio_set_level(LED_BUILTIN, !gpio_get_level(LED_BUILTIN));    
}*/

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {

    //gpio_set_level(LED_BUILTIN, !gpio_get_level(LED_BUILTIN));

	if (timer != NULL) {
		LedStateMsg.data = hcsr_task();
		RCSOFTCHECK(rcl_publish(&led_state_publisher, &LedStateMsg, NULL));
		printf("send ok\n");
		//LedStateMsg.data++;
	}

    if (timer == NULL) {
        return;
    }

	printf("timer passed\n");

    // Use linear.x for forward value and angular.z for rotation
    float linear = constrain(msg.linear.x, -1, 1);
    float angular = constrain(msg.angular.z, -1, 1);

    // This robot is an RC tank and uses a differential drive (skid steering).
    // Calculate the speed of left and right motors. Simple version without wheel distances.
    // Taken from https://hackernoon.com/unicycle-to-differential-drive-courseras-control-of-mobile-robots-with-ros-and-rosbots-part-2-6d27d15f2010#1e59
    float left = (linear - angular) / 2.0f;
    float right = (linear + angular) / 2.0f;

    // Then map those values to PWM intensities. PWM_MOTOR_MAX = full speed, PWM_MOTOR_MIN = the minimal amount of power at which the motors begin moving.
    uint16_t pwmLeft = (uint16_t) fmap(fabs(left), 0, 1, PWM_MOTOR_MIN, PWM_MOTOR_MAX);
    uint16_t pwmRight = (uint16_t) fmap(fabs(right), 0, 1, PWM_MOTOR_MIN, PWM_MOTOR_MAX);

    // Each wheel has a channel for forwards and backwards movement
    ledc_set_duty(PWM_MODE, PWM_LEFT_FORWARD, pwmLeft * (left > 0));
    ledc_set_duty(PWM_MODE, PWM_LEFT_BACKWARD, pwmLeft * (left < 0));
    ledc_set_duty(PWM_MODE, PWM_RIGHT_FORWARD, pwmRight * (right > 0));
    ledc_set_duty(PWM_MODE, PWM_RIGHT_BACKWARD, pwmRight * (right < 0));

    ledc_update_duty(PWM_MODE, PWM_LEFT_FORWARD);
    ledc_update_duty(PWM_MODE, PWM_LEFT_BACKWARD);
    ledc_update_duty(PWM_MODE, PWM_RIGHT_FORWARD);
    ledc_update_duty(PWM_MODE, PWM_RIGHT_BACKWARD);

    printf("%d, %d %d, %d, %d %d, %f, %f\n", pwmLeft, left > 0, left < 0, pwmRight, right > 0, right < 0, left, right);
}


// Helper functions
// -------------------------------------------------------------------

float fmap(float val, float in_min, float in_max, float out_min, float out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void micro_ros_task(void * arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

	// Static Agent IP and port can be used instead of autodisvery.
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
	//RCCHECK(rmw_uros_discover_agent(rmw_options));

	// create init_options
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "blink_on_input", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&led_state_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"led_state"));
	
		// create subscriber 
	RCCHECK(rclc_subscription_init_default(
		&led_input_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"led_input"));
	
	  // create subscriber
    rcl_subscription_t subscriber;
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"));

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
	RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &led_input_subscriber,&LedInputMsg,
		&subscription_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &cmd_vel_callback, ON_NEW_DATA));

	//msg.data = 0;

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(10000);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&led_state_publisher, &node));
	RCCHECK(rcl_subscription_fini(&led_input_subscriber, &node));
	RCCHECK(rcl_subscription_fini(&subscriber, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

/*/ Function forward declarations
void setupPins();
void setupRos();
void cmd_vel_callback(const void *msgin);
void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
float fmap(float val, float in_min, float in_max, float out_min, float out_max);*/

// We don't really need the callback, because msg is set anyway
void cmd_vel_callback(const void *msgin) {
//    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *) msgin;
//    printf("Message received: %f %f\n", msg->linear.x, msg->angular.z);
}

// Each frame, check msg data and set PWM channels accordingly



void app_main(void)
{   

//setupPins();

#ifdef UCLIENT_PROFILE_UDP
    // Start the networking if required
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif  // UCLIENT_PROFILE_UDP

    //pin micro-ros task in APP_CPU to make PRO_CPU to deal with wifi:
    xTaskCreate(micro_ros_task, 
            "uros_task", 
            CONFIG_MICRO_ROS_APP_STACK, 
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO, 
            NULL); 
}













