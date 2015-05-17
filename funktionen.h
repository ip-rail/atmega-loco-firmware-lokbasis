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
extern void warte_ms(unsigned int wartezeit);
extern void remcrlf(char *crlftext);
extern void init_gpios();

#endif /* FUNKTIONEN_H_ */
