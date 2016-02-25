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
#define WLAN_EXTRA		// Wiznet Wiz610wi (oder anderes WLAN-Modul)
//#define WLAN_RASPI			// Raspi (WLAN über selbe UART wie Bootloader)
#define PHB01_MOTOR1		// Motorcontroller PHB01: H-Brücke wird für Motor 1 verwendet
//#define PHB01_MOTOR2		// Motorcontroller PHB01: eine 2. H-Brücke wird für Motor 2 verwendet
//#define UART_NR_RFID 1	// uart1 für RFID
//#define LEDC_TLC59116	1	//LED-Controller TLC59116 ist vorhanden (es kann derzeit nur einer davon verwendet werden. entweder PCA9622 oder TLC59116!!)
#define LEDC_PCA9622	1	//LED-Controller PCA9622 ist vorhanden



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
//#define MOTOR1_ADC 	PF7 	// Input, ADC7 Motor 1
//#define MOTOR2_ADC 	PF6 	// Input, ADC6 Motor 2

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

#define PIN_MOTOR PINA

#define MOTOR_PWMF_STD	8	// Motor-PWM-Frequenz Standardwert: Mode 8: 15656Hz


//ADC
#define ADC_SCHIENE PF0 		// Input, ADC Schienenspannung


// GPIOs frei für Schalt-Funktionen (Licht usw.)
#define GPIO_USABLE_PORT_A	0
#define GPIO_USABLE_PORT_B	143
#define GPIO_USABLE_PORT_C	255
#define GPIO_USABLE_PORT_D	240
#define GPIO_USABLE_PORT_E	56
#define GPIO_USABLE_PORT_F	0
#define GPIO_USABLE_PORT_G	23

#endif	// HW_UC02


// I2C

// I2C clock in Hz. Einstellung aus File twimaster.c
// #define SCL_CLOCK  100000L	//100kHz
#define SCL_CLOCK  50000L	//50kHz

// I2C LED-Controller PCA9622,TLC59116

//ACHTUNG: i2c-Adressen werden in der AVR-Software schon um ein bit nach links verschoben angegeben (8bit statt 7bit),
//         damit man nur mehr das Lese/Schreib-bit dazuaddieren muss!!

#if defined( LEDC_TLC59116 )

	#define LEDC_I2C_RESET   0xD6		// 0xD6 = Software Reset I2C Bus Adresse
	#define LEDC1  0xC0      			// I2C-Adresse des LED-Controller
	#define LEDC_MODE2_BLINK	0x20	// Mode 2 = Blinken
	#define LEDC_MODE2_DIMM		0		// Mode 2 = Dimmen
	
#endif	// LEDC_TLC59116

#if defined( LEDC_PCA9622 )

	#define LEDC_I2C_RESET   0x06		// 0x03 = Software Reset Adresse der PLED01 (PCA9622) LED-Controller -> 0x03 << 1 = 0x06
	#define LEDC1  0x1c      			// I2C-Adresse des LED-Controller 0x0e -> 0x1c
	#define LEDC_MODE2_BLINK	0x25	// Mode 2 = Blinken
	#define LEDC_MODE2_DIMM		5		// Mode 2 = Dimmen
	
#endif	// LEDC_PCA9622

#define LEDC_MODE1	1		// Mode 1: ALLCALL aktiv(1) (nur bit0 auf 1, alle andern auf 0), für beide Controller gleich

// Adressen der Register im PCA9622 und TLC59116
#define PCA9622_ADDR_MODE1			0x00
#define PCA9622_ADDR_MODE2			0x01
#define PCA9622_ADDR_PWM0			0x02	// PWM (Helligkeits)-Register 1. LED
#define PCA9622_ADDR_GRPPWM			0x12	// Gruppen-PWM (Helligkeit)
#define PCA9622_ADDR_GRPFREQ		0x13	// Gruppen-Blink-Frequenz
#define PCA9622_ADDR_LEDOUT0		0x14	// LED driver output state register (LED 0 - 3)
#define PCA9622_ADDR_LEDOUT1		0x15	// LED driver output state register (LED 4 - 7)
#define PCA9622_ADDR_LEDOUT2		0x16	// LED driver output state register (LED 8 - 11)
#define PCA9622_ADDR_LEDOUT3		0x17	// LED driver output state register (LED 12 - 15)





#if defined( WLAN_RASPI )

// UART Definitionen
#define UART_BAUDRATE_RFID	9600	// RFID-Verbindung
//#define UART_BAUDRATE_WLAN	115200	// WLAN-Verbindung
#define UART_BAUDRATE_WLAN	38400	// WLAN-Verbindung - nur für test

// WLAN
#define WLAN_UART_NR	0	//UART0
#define WLAN_RX 	PE0 	// Input, Atmega RX, Wiznet Data out
#define WLAN_TX 	PE1 	// Output, Atmega TX, Wiznet Data in
#define DDR_WLAN	DDRE
#define PORT_WLAN	PORTE
#define PIN_WLAN	PINE

#endif	// WLAN_RASPI


#if defined( WLAN_EXTRA )

#define UART_BAUDRATE_RFID	9600
#define UART_BAUDRATE_WLAN	38400

#define WLAN_UART_NR	1	//UART1
#define WLAN_RX 	PD2 	// Input, Atmega RX, Wiznet Data out
#define WLAN_TX 	PD3 	// Output, Atmega TX, Wiznet Data in
#define DDR_WLAN	DDRD
#define PORT_WLAN	PORTD
#define PIN_WLAN	PIND

#endif	// WLAN_EXTRA

#define UART_SETTING_RFID ((F_CPU/16L/UART_BAUDRATE_RFID)-1)	// Berechnung der UART Konfigurationsparameter
#define UART_SETTING_WLAN ((F_CPU/16L/UART_BAUDRATE_WLAN)-1)

#endif	// LOKBASIS_HWDEF_H_
