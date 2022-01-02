/*
    IoT Simplified (iotsimplified.com)
    
    Sensor Driver for the HC-SR04 Ultrasonic Range Finding Sensor

    Written by Ahmed Al Bayati
    


    Unless required by applicable law or agreed to in writing, this
    software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
    CONDITIONS OF ANY KIND, either express or implied.
    
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"

#include "hc-sr04_driver.h"



/*#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <math.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

#include <driver/gpio.h>*/
#include <driver/ledc.h>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif


#define HCSR_TRIG_GPIO_NUM GPIO_NUM_23
#define HCSR_ECHO_GPIO_NUM GPIO_NUM_22
#define LED_BUILTIN 2
#define HCSR_HIGH 1
#define HCSR_LOW 0


static const char *TAG = "hc-sr04_driver";


/*#define FRAME_TIME 100 // 1000 / FRAME_TIME = FPS
#define SLEEP_TIME 10*/

// PINS
//#define LED_BUILTIN 33
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
#define PWM_MODE LEDC_HIGH_SPEED_MODE

// Function Prototypes
int hcsr_setup_pins(); //Setup all the pins and such to get the GPIO ready to be used
esp_err_t hcsr_send_trig_signal(); //Send a 10uS trig signal HIGH to the sensor
int hcsr_echo_pulse_read(); //Read the returned pulse and update a variable



// Function Def's

void cmd_vel_callback(const void *msgin);

int hcsr_setup_pins()
{
    ESP_LOGI(TAG,"Setting up GPIO Pins");
    //Config the Trig Pin
    gpio_pad_select_gpio(HCSR_TRIG_GPIO_NUM);
    gpio_set_direction(HCSR_TRIG_GPIO_NUM,GPIO_MODE_OUTPUT);

    //Config the Echo Pin
    gpio_pad_select_gpio(HCSR_ECHO_GPIO_NUM);
    gpio_set_direction(HCSR_ECHO_GPIO_NUM,GPIO_MODE_INPUT);

    gpio_reset_pin(LED_BUILTIN);
    gpio_set_direction(LED_BUILTIN, GPIO_MODE_INPUT_OUTPUT);

    //ESP_LOGI(TAG,"GPIO Pins Setup Completed");


    //gpio_reset_pin(LED_BUILTIN);
    //gpio_set_direction(LED_BUILTIN, GPIO_MODE_INPUT_OUTPUT);

    // Configure timer
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQUENCY,
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    // Configure 4 PWM channels and assign output pins
    ledc_channel_config_t ledc_channel[4] = {
        {
            .channel    = PWM_LEFT_FORWARD,
            .duty       = 0,
            .gpio_num   = PIN_LEFT_FORWARD,
            .speed_mode = PWM_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_1
        },
        {
            .channel    = PWM_LEFT_BACKWARD,
            .duty       = 0,
            .gpio_num   = PIN_LEFT_BACKWARD,
            .speed_mode = PWM_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_1
        },
        {
            .channel    = PWM_RIGHT_FORWARD,
            .duty       = 0,
            .gpio_num   = PIN_RIGHT_FORWARD,
            .speed_mode = PWM_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_1
        },
        {
            .channel    = PWM_RIGHT_BACKWARD,
            .duty       = 0,
            .gpio_num   = PIN_RIGHT_BACKWARD,
            .speed_mode = PWM_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_1
        },
    };

    for (int i = 0; i < 4; i++) {
        ledc_channel_config(&ledc_channel[i]);
    }

    ESP_LOGI(TAG,"GPIO Pins Setup Completed NOW!");

    return 0;
}

esp_err_t hcsr_send_trig_signal()
{
    //ESP_LOGI(TAG,"Sending Trig Signal to HCSR04 Sensor");
    esp_err_t err;
    //Signal sent needs to be on HIGH for 10 uS
    err = gpio_set_level(HCSR_TRIG_GPIO_NUM,HCSR_LOW);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG,"ERROR: Trig was unable to be sent");
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(0.002));

    err = gpio_set_level(HCSR_TRIG_GPIO_NUM,HCSR_HIGH);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG,"ERROR: Trig was unable to be sent");
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(0.01));

    err = gpio_set_level(HCSR_TRIG_GPIO_NUM,HCSR_LOW);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG,"ERROR: Trig was unable to be sent");
        return err;
    }

    //ESP_LOGI(TAG,"Trig Signal Sending Completed");
    return err;

}

int hcsr_echo_pulse_read()
{
    // ESP_LOGI(TAG,"Echo Reading");
    //Pulse Width Input - So we need to read how long is the input held up HIGH and return that value
    bool no_Signal = true;
    int pulse_Width = 0; // in uS
    int counter = 0;
    while(no_Signal)
    {
        if(gpio_get_level(HCSR_ECHO_GPIO_NUM) == HCSR_HIGH)
        {   
            no_Signal = false;
            while(gpio_get_level(HCSR_ECHO_GPIO_NUM) == HCSR_HIGH)
            {
                //Calculate the time in Microseconds
                pulse_Width += 1;
                vTaskDelay(pdMS_TO_TICKS(0.001));
            }
        }
        else if(counter > 100)
        {
            hcsr_send_trig_signal();
            counter = 0;
        }
        counter++;
        
    }
    // ESP_LOGI(TAG,"Echo Reading Completed");
    return pulse_Width;
}

double hcsr_get_distance_in()
{
    esp_err_t err;
    int pulse_readout = 0;
    double distance = 0.0;
    
    err = hcsr_send_trig_signal();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG,"ERROR: within hcsr_send_trig_signal()");
        esp_restart();
    }

    pulse_readout = hcsr_echo_pulse_read();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG,"ERROR: within hcsr_send_trig_signal()");
        esp_restart();
    }

    ESP_LOGI(TAG,"The Value of the Pulse Width is: %d",pulse_readout);
    distance = ((double)pulse_readout / 148.0) * 10;
    //ESP_LOGI(TAG,"Distance in Inches: %f",distance);
    return distance;
}

 
double hcsr_caliberate_sensor()
{
    ESP_LOGI(TAG,"Caliberating Sensor");

    double distance = 0.0;

    vTaskDelay(pdMS_TO_TICKS(10000)); // Wait 10 seconds for sensor stability

    for(int i = 0; i < 10; i++)
    {
        // Get 10 readings and average them
        distance += hcsr_get_distance_in();
        vTaskDelay(pdMS_TO_TICKS(HCSR_MEASUREMENT_CYCLE_MS));
    }

    return distance / 10.0;
}