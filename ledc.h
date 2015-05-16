/*
 * ledc.h
 *
 *  Created on: 16.05.2015
 *      Author: Michael Brunnbauer
 *
 *      I2C LED-Controller PCA9622,TLC59116
 */

#ifndef LEDC_H_
#define LEDC_H_

extern void ledcontrol_reset();
extern void ledcontrol_init(uint8_t slaveaddr);
extern void ledc_led_setpwm(int slaveaddr, uint8_t lednumber, uint8_t value);


#endif // LEDC_H_ 
