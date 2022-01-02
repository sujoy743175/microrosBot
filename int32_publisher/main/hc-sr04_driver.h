/*
    IoT Simplified (iotsimplified.com)
    
    Sensor Driver for the HC-SR04 Ultrasonic Range Finding Sensor


    Written by Ahmed Al Bayati
    
*/


#ifndef __HCSR04_DRIVER_H__
#define __HCSR04_DRIVER_H__

#define HCSR_MEASUREMENT_CYCLE_MS 90


int hcsr_setup_pins(); //Setup all the pins and such to get the GPIO ready to be used
esp_err_t hcsr_send_trig_signal(); //Send a 10uS trig signal HIGH to the sensor
int hcsr_echo_pulse_read(); //Read the returned pulse and update a variable
double hcsr_get_distance_in(); // Returns the distance in inches
double hcsr_caliberate_sensor(); // Caliberate the Sensor

#endif