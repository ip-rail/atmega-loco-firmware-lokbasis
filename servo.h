/*
 * servo.h
 *
 *  Created on: 19.06.2015
 *      Author: Michael Brunnbauer
 */

#ifndef SERVO_H_
#define SERVO_H_

// Der Prescaler muss so gewählt werden, dass der Ausdruck
// für MILLISEC_BASE einen Wert kleiner als 128 (bei 8bit-Timer) oder  32768 (bei 16bit-Timer) ergibt
// MILLISEC_BASE ist der Timerwert, der 1 Millisekunde Zeitdauer ergeben soll.

#define PRESCALER      1

#define MILLISEC_BASE  ( F_CPU / PRESCALER / 1000 )		// 1ms
#define CENTER         ( MILLISEC_BASE / 2 )

#define SERVOCOUNTMAX      6	// vorerst max. 6 Servo-Signale verwendbar

// folgende Werte sind nicht mehr konstant und müssen daher berechnet werden:
//#define SERVOCOUNT      2
//#define SERVOSLEEP     20 - (SERVOCOUNT * 2) - 1	// grob, muss nicht genau sein
//#define SERVO_DDR      DDRB
//#define SERVO_PORT     PORTB

extern uint8_t servo_mode;				// 0: für jedes Servosignal wird ein GPIO verwendet (vordefinieren welche!). 1: alle Signale nur an 1 GPIO ausgeben (Port,Pin mit Index 0), an dem ein Kreiszähler-IC hängt
extern uint8_t servo_count;				// wieviele Servosignale ausgeben (max. 6-8?)
extern uint8_t servoPort[SERVOCOUNTMAX];	//Port (Buchstabe Großschrift) und Pin-Nummer der GPIOs, die für Servos verwendet werden (verwendet werden Index 0 bis SERVOCOUNTMAX-1)
extern uint8_t servoPin[SERVOCOUNTMAX];	//Port und Pin gehören je Index zusammen
extern uint8_t servo_sleep;				// grober Wert, wieviel von den 20ms Servo-Intervall gewarret werden muss
extern volatile unsigned int servoValue[SERVOCOUNTMAX];
extern volatile uint8_t servo_error;	// error bei servo IO

extern void initServo();
extern void Servo_stop();
extern void Servo_start();

#endif /* SERVO_H_ */
