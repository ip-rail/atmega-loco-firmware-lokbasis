/*
 * funktionen.h
 *
 *  Created on: 08.01.2011 - 02.05.2015
 *      Author: Michael Brunnbauer
 */

#ifndef FUNKTIONEN_H_
#define FUNKTIONEN_H_

extern void init_timer5();
extern void init_uart(uint8_t uartnr, const unsigned int uartsetting);
extern void init_motorctrl(void);
extern void init_pwm(char freq_pwm);
extern void motor_sleep(void);
extern void motor_wakeup(void);
extern void motor_reset(void);
extern void checkMotorStatus();
extern void warte_ms(unsigned int wartezeit);
extern void remcrlf(char *crlftext);
extern void init_gpios();
extern uint8_t getUsableGPIOs(char port);
extern uint8_t filterGPIOMask(char port, uint8_t mask);
extern uint8_t getGPIOs(char port);
extern uint8_t getGPIOValues(char port);
extern void setGPIOPin(char port, uint8_t pinnr, uint8_t pinval);
extern void sendHWinfo(char * txtbuffer);

#endif /* FUNKTIONEN_H_ */
