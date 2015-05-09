/*
 * lokbasis_hwdef.h
 *
 *  Created on: 22.10.2011 - 08.05.2015
 *      Author: Michael Brunnbauer
 *
 *  Hardwaredefinitionen für Lokbasis Hardware
 *
 *
 *
 */

#ifndef LOKBASIS_HWDEF_H_
#define LOKBASIS_HWDEF_H_

#ifndef __AVR_ATmega2561__
#define __AVR_ATmega2561__
#endif



// Konfiguration
#define HW_UC02				// WLANcroc MC-Board
//#define WLAN_WIZNET		// Wiznet Wiz610wi
#define WLAN_RASPI			// Raspi
#define PHB01_MOTOR1		// Motorcontroller PHB01: H-Brücke wird für Motor 1 verwendet
//#define PHB01_MOTOR2		// Motorcontroller PHB01: eine 2. H-Brücke wird für Motor 2 verwendet
#define UART_NR_WLAN 0		// uart0 für WLAN
//#define UART_NR_RFID 1	// uart1 für RFID
//UART Konfiguration: siehe uart.h
//-------------------------------------------------------------------------------------------


//UC02 Wlancroc-Standard MC Modul ATMega2561
#if defined( HW_UC02 )

// Testpin für Toggle-Signal 0,5Hz
#define DDR_TESTSIGNAL	DDRD
#define PORT_TESTSIGNAL	PORTD
#define TESTSIGNAL	PD7
#define TESTSIGNAL2	PD6

// Motorcontroller PHB01
//#define MOTOR1_ADC 	PF7 	// Input, ADC6 Motor 1
//#define MOTOR2_ADC 	PF6 	// Input,ADC7 Motor 2

#define MOTOR1_DIR 	PA0 	// Output, Motor1 Richtung
#define MOTOR2_DIR 	PA4 	// Output, Motor1 Richtung
#define MOTOR1_PWM 	PB5 	// Output, 1. Motor PWM
#define MOTOR2_PWM 	PB6 	// Output, 2. Motor PWM
#define MOTOR1_OCR	OCR1A	//
#define MOTOR2_OCR	OCR1B	//
#define MOTOR1_RESET PA1 	// Output, Motor1 Reset
#define MOTOR2_RESET PA5  	// Output, Motor2 Reset
#define MOTOR1_FF1 	PA2 	// Input, Motor1 Error1
#define MOTOR1_FF2 	PA3 	// Input, Motor1 Error2
#define MOTOR2_FF1 	PA6 	// Input, Motor2 Error1
#define MOTOR2_FF2 	PA7 	// Input, Motor2 Error2

#define DDR_MOTOR		DDRA
#define DDR_MOTOR_PWM	DDRB
#define PORT_MOTOR		PORTA
#define PORT_MOTOR_PWM	PORTB

#define PIN_MOTOR PINC


//ADC
#define ADC_SCHIENE PF0 		// Input, ADC Schienenspannung

// I2C LED-Controller
//#define LEDC_TLC59116	1			//LED-Controller TLC59116 ist vorhanden
#define I2C_RESET   0xD6			// 0xD6 = Software Reset I2C Bus Adresse
#define LEDC_MODE2_BLINK	32		// Mode 2 = Blinken
#define LEDC_MODE2_DIMM		0		// Mode 2 = Dimmen
#define LEDC1  0xC0      			// I2C-Adresse des LED-Controller

#endif	// HW_UC02


#if defined( WLAN_RASPI )

// UART Definitionen
#define UART_BAUDRATE_RFID	9600	// RFID-Verbindung
//#define UART_BAUDRATE_WLAN	115200	// WLAN-Verbindung
#define UART_BAUDRATE_WLAN	38400	// WLAN-Verbindung - nur für test

#define UART_SETTING_RFID ((F_CPU/16L/UART_BAUDRATE_RFID)-1)	// Berechnung der UART Konfigurationsparameter
#define UART_SETTING_WLAN ((F_CPU/16L/UART_BAUDRATE_WLAN)-1)

// WLAN
#define WLAN_RX 	PE0 	// Input, Atmega RX, Wiznet Data out
#define WLAN_TX 	PE1 	// Output, Atmega TX, Wiznet Data in
#define DDR_WLAN	DDRE
#define PORT_WLAN	PORTE
#define PIN_WLAN	PINE

#endif	// WLAN_RASPI


#endif	// LOKBASIS_HWDEF_H_
