/*
 * funktionen.c
 *
 *  Created on: 08.01.2011 - 02.05.2015
 *      Author: Michael Brunnbauer
 *
 *      diverse Funktionen - besonders für Hardware
 */

#include <avr/io.h>
#include <string.h>		// für "strcmp"
#include <stdio.h>		// für "sprintf"
#include <stdlib.h>		// für "itoa"
#include <util/delay.h>	// für delay_ms()
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "lokbasis_hwdef.h"
#include "eedata.h"
#include "main.h"
#include "servo.h"
#include "wlan.h"
#include "uart.h"
#include "funktionen.h"
#include "eedata.h"

//char gpioport_all[] = { 'B', 'C', 'D', 'E', 'G' };
//char gpioport[] = { 'B', 'D', 'E', 'G' };


void init_timer5()
{
	// timer 5 als interner timer mit 244Hz (4ms) (passt für atmege2560 und 2561) ------------------------
	TCNT5 = 0;				// set timer counter initial value
	OCR5A = 255;			// TOP value setzen

	setbit(TCCR5B,CS52);	// start timer with prescaler 256 -> 244Hz = 4ms
	clearbit(TCCR5B,CS51);
	clearbit(TCCR5B,CS50);

	clearbit(TCCR5B,WGM53);	// Mode 4 (0100) OCR5A = 255, damit nur bis 255 gezählt wird
	setbit(TCCR5B,WGM52);
	clearbit(TCCR5A,WGM51);
	clearbit(TCCR5A,WGM50);

	setbit(TIMSK5,OCIE5A);	// enable output compare match interrupt
}


//-----------------------------------------------------------------------------------

void init_uart(uint8_t uartnr, const unsigned int uartsetting)
{
	switch (uartnr)
	{
	case 0:
		UART_TxHead = 0;
		UART_TxTail = 0;
		UART_RxHead = 0;
		UART_RxTail = 0;

		UBRR0H = (unsigned char) (uartsetting >> 8);	// Baudrate einstellen: H muss vor L geschrieben werden!!
		UBRR0L = (unsigned char) (uartsetting);
		/*
		UCSR0B = (1<<RXEN0) | (1<<TXEN0);	// senden & empfangen einschalten
		UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);	// asynchron 8N1
		UCSR0B |= (1<<RXCIE0);	// RX Interrupt einschalten
		*/

		setbit(UCSR0B,RXEN0);	// empfangen einschalten
		setbit(UCSR0B,TXEN0);	// senden einschalten
		clearbit(UCSR0B,UCSZ02);// UCSZ02 -> 0 für 8bit

		// Asynchron, 8N1
		clearbit(UCSR0C,UMSEL01);	// asynchrone Übertragung
		clearbit(UCSR0C,UMSEL00);	// asynchrone Übertragung
		setbit(UCSR0C,UCSZ00);		// 8bit
		setbit(UCSR0C,UCSZ01);
		clearbit(UCSR0C,UPM00);		// no parity
		clearbit(UCSR0C,UPM01);
		clearbit(UCSR0C,USBS0);		// 1 stopbit
		setbit(UCSR0B,RXCIE0);	// RX Interrupt einschalten

		break;

//#if defined( ATMEGA_USART1 )
	case 1:

		UART1_TxHead = 0;
		UART1_TxTail = 0;
		UART1_RxHead = 0;
		UART1_RxTail = 0;

		UBRR1H = (unsigned char) (uartsetting >> 8);	// Baudrate einstellen: H muss vor L geschrieben werden!!
		UBRR1L = (unsigned char) (uartsetting);
		/*
		UCSR1B = (1<<RXEN1) | (1<<TXEN1);	// senden & empfangen einschalten
		UCSR1C = (1<<UCSZ10) | (1<<UCSZ11);	// asynchron 8N1
		UCSR1B |= (1<<RXCIE1);	// RX Interrupt einschalten
		 */
		setbit(UCSR1B,RXEN1);	// empfangen einschalten
		setbit(UCSR1B,TXEN1);	// senden einschalten
		clearbit(UCSR1B,UCSZ12);// UCSZ12 -> 0 für 8bit

		// Asynchron, 8N1
		clearbit(UCSR1C,UMSEL11);	// asynchrone Übertragung
		clearbit(UCSR1C,UMSEL10);	// asynchrone Übertragung
		setbit(UCSR1C,UCSZ10);		// 8bit
		setbit(UCSR1C,UCSZ11);
		clearbit(UCSR1C,UPM10);		// no parity
		clearbit(UCSR1C,UPM11);
		clearbit(UCSR1C,USBS1);		// 1 stopbit
		setbit(UCSR1B,RXCIE1);	// RX Interrupt einschalten


		break;
//#endif // ATMEGA_USART1

	default:
		break;
	}
}


// Motorsteuerung initialisieren

void init_motorctrl(void)
{
	motor_cfg = eeprom_getMotorConfig();


#if defined( HW_UC02 )	//WLANcroc H-Brücke

	#if defined( PHB01_MOTOR1 )
		DDR_MOTOR |= (1<<MOTOR1_DIR) | (1<<MOTOR1_RESET);	// als Ausgänge setzen - nur 1 H-Brücke

		if (motor_cfg & MOTOR_CFG_CHECK_ERR_HB1)
		{
			DDR_MOTOR &= ~((1<<MOTOR1_FF1) | (1<<MOTOR1_FF2));	// Fehlerpins als Eingänge setzen
		}

		//PORT_MOTOR |= (1<<MOTOR1_DIR);	// Richtung auf 1 setzen // je nach Motorcontroller
		PORT_MOTOR &= ~(1<<MOTOR1_DIR);   	// Motor-Richtung auf 0 // bei diesem Controller: vorwärts
		PORT_MOTOR |= (1<<MOTOR1_RESET);	// Reset auf HI

	#endif

	#if defined( PHB01_MOTOR2 )
		if (motor_cfg & MOTOR_CFG_USE_HBRIDGE2)
		{
			DDR_MOTOR |= (1<<MOTOR2_DIR) | (1<<MOTOR2_RESET);	// als Ausgänge setzen

			if (motor_cfg & MOTOR_CFG_CHECK_ERR_HB2)
			{
				DDR_MOTOR &= ~((1<<MOTOR2_FF1)  | (1<<MOTOR2_FF2));	// Fehlerpins als Eingänge setzen
			}

			//PORT_MOTOR |= (1<<MOTOR2_DIR);	// Richtung auf 1 setzen // je nach Motorcontroller
			PORT_MOTOR &= ~(1<<MOTOR2_DIR);   	// Motor-Richtung auf 0 // bei diesem Controller: vorwärts
			PORT_MOTOR |= (1<<MOTOR2_RESET);	// Reset auf HI
		}
	#endif

		// Timer 1 für Motor-PWM initialisieren --------------------------------------
		motor_pwmf = eeprom_getMotorPWMf();
		if ((motor_pwmf < 1) || (motor_pwmf > 9)) { motor_pwmf = MOTOR_PWMF_STD; } // muss im Berech on 1 bis 9
		init_pwm(motor_pwmf);	// Standardwert: Mode 8: 15656Hz 9bit prescaler 1  - Timer 1 für Motor-PWM initialisieren

#endif
}



void init_pwm(char freq_pwm)	// Timer 1 für Motor-PWM initialisieren
{

	speedstep_korrektur = 0;	// gibt an, ob speed *2 oder *4 genommen werden muss (8/9bit), globale varaible für speed-stellung
	//clearbit(TIMSK1,OCIE1A);	// Compare Match Interrupt abdrehen (wird für Motor-Messung verwendet) - war für motor-mess-test

	if ((freq_pwm < 1) | (freq_pwm > 9)) { freq_pwm = 5; }

	/*
	Frequenztabelle (Phase correct PWM):
	1: 122Hz		8bit:256, 10bit:64
	2: 245Hz		9bit:64
	3: 490Hz		8bit:64
	4: 977Hz		10bit:8
	5: 1957Hz		9bit:8
	6: 3921Hz		8bit:8
	7: 7820Hz		10bit:1
	8: 15656Hz		9bit:1
	9: 31372Hz		8bit:1

    Phase correct PWM: f-pwm = 16MHz / (2 * prescaler * TOP )	TOP: 10bit=1023, 9bit=511, 8bit=255
	*/


	#if defined( PHB01_MOTOR1 )
		DDR_MOTOR_PWM |= (1<<MOTOR1_PWM);	// als Ausgang setzen - 1. H-Brücke
		MOTOR1_OCR = 0;
	#endif

	#if defined( PHB01_MOTOR2 )
		if (motor_cfg & MOTOR_CFG_USE_HBRIDGE2)
		{
			DDR_MOTOR_PWM |= (1<<MOTOR2_PWM);	// als Ausgang setzen - 2. H-Brücke
			MOTOR2_OCR = 0;
		}

	#endif

	//clearbit(PRR0,PRTIM1);	// Power Reduction für Timer 1 deaktivieren
	setbit(TCCR1A,COM1A1);		// Compare Output mode for channel A = non-inverted
	clearbit(TCCR1A,COM1A0);	// Compare Output mode for channel A = non-inverted
	setbit(TCCR1A,COM1B1);		// Compare Output mode for channel B = non-inverted
	clearbit(TCCR1A,COM1B0);	// Compare Output mode for channel B = non-inverted
	clearbit(TCCR1B,WGM13);		// Phase correct PWM, waveform generation bits - die oberen 2 sind für alle gleich!
	clearbit(TCCR1B,WGM12);

	switch (freq_pwm)
	{
		case 1:	// 122Hz 8bit:256, 10bit:64*
			setbit(TCCR1A,WGM11);
			setbit(TCCR1A,WGM10);
			clearbit(TCCR1B,CS12);		// prescaler = 64
			setbit(TCCR1B,CS11);		// prescaler = 64
			setbit(TCCR1B,CS10);		// prescaler = 64
		break;

		case 2:	// 245Hz 9bit:64
			setbit(TCCR1A,WGM11);
			clearbit(TCCR1A,WGM10);
			clearbit(TCCR1B,CS12);		// prescaler = 64
			setbit(TCCR1B,CS11);		// prescaler = 64
			setbit(TCCR1B,CS10);		// prescaler = 64
			speedstep_korrektur = 1;
		break;

		case 3:	// 490Hz 8bit:64
			clearbit(TCCR1A,WGM11);
			setbit(TCCR1A,WGM10);
			clearbit(TCCR1B,CS12);		// prescaler = 64
			setbit(TCCR1B,CS11);		// prescaler = 64
			setbit(TCCR1B,CS10);		// prescaler = 64
			speedstep_korrektur = 2;
		break;

		case 4:	// 977Hz 10bit:8
			setbit(TCCR1A,WGM11);
			setbit(TCCR1A,WGM10);
			clearbit(TCCR1B,CS12);		// prescaler = 8
			setbit(TCCR1B,CS11);		// prescaler = 8
			clearbit(TCCR1B,CS10);		// prescaler = 8
		break;

		case 5:	// 1957Hz 9bit:8
			setbit(TCCR1A,WGM11);
			clearbit(TCCR1A,WGM10);
			clearbit(TCCR1B,CS12);		// prescaler = 8
			setbit(TCCR1B,CS11);		// prescaler = 8
			clearbit(TCCR1B,CS10);		// prescaler = 8
			speedstep_korrektur = 1;
		break;

		case 6:	// 3921Hz 8bit:8
			clearbit(TCCR1A,WGM11);
			setbit(TCCR1A,WGM10);
			clearbit(TCCR1B,CS12);		// prescaler = 8
			setbit(TCCR1B,CS11);		// prescaler = 8
			clearbit(TCCR1B,CS10);		// prescaler = 8
			speedstep_korrektur = 2;
		break;

		case 7:	// 7820Hz 10bit:1
			setbit(TCCR1A,WGM11);
			setbit(TCCR1A,WGM10);
			clearbit(TCCR1B,CS12);		// prescaler = 1
			clearbit(TCCR1B,CS11);		// prescaler = 1
			setbit(TCCR1B,CS10);		// prescaler = 1
		break;

		case 8:	// 15656Hz 9bit:1
			setbit(TCCR1A,WGM11);
			clearbit(TCCR1A,WGM10);
			clearbit(TCCR1B,CS12);		// prescaler = 1
			clearbit(TCCR1B,CS11);		// prescaler = 1
			setbit(TCCR1B,CS10);		// prescaler = 1
			speedstep_korrektur = 1;
		break;

		case 9:	// 31372Hz 8bit:1
			clearbit(TCCR1A,WGM11);
			setbit(TCCR1A,WGM10);
			clearbit(TCCR1B,CS12);		// prescaler = 1
			clearbit(TCCR1B,CS11);		// prescaler = 1
			setbit(TCCR1B,CS10);		// prescaler = 1
			speedstep_korrektur = 2;
		break;

		default:	// 1957Hz 9bit:8
			setbit(TCCR1A,WGM11);
			clearbit(TCCR1A,WGM10);
			clearbit(TCCR1B,CS12);		// prescaler = 8
			setbit(TCCR1B,CS11);		// prescaler = 8
			clearbit(TCCR1B,CS10);		// prescaler = 8
			speedstep_korrektur = 1;
		break;
	}
	// speed muss für 8bit/9bit pwm um 1 bzw. 2 bit geshiftet werden, damit der speedwert passt (also halbiert bzw. geviertelt) -> in set_speed()

	// Compare Match Interrupt aufdrehen (für Motor Messung)
	// setbit(TIMSK1,OCIE1A); war für motor-mess-test
}


// Motorcontroller PHB01 in Schlafmodus versetzen
void motor_sleep(void)
{
	PORT_MOTOR &= ~(1<<MOTOR1_RESET);	// Reset auf LO

	//TODO: Motor2 fehlt noch
	#if defined( PHB01_MOTOR2 )
		if (motor_cfg & MOTOR_CFG_USE_HBRIDGE2) { PORT_MOTOR &= ~(1<<MOTOR2_RESET); }
	#endif
}

// Motorcontroller PHB01 aus Schlafmodus aufwecken (dauert ca. 3ms)
void motor_wakeup(void)
{
	PORT_MOTOR |= (1<<MOTOR1_RESET);	// Reset auf HI

	#if defined( PHB01_MOTOR2 )
		if (motor_cfg & MOTOR_CFG_USE_HBRIDGE2) { PORT_MOTOR |= (1<<MOTOR2_RESET); }
	#endif

	//warte_ms(4);	// TODO: Wartezeit sinnvoll?
}

// Motorcontroller PHB01 Reset zum Fehlerquitieren (wie lange auf LOW: >0,1µs <3,5µs)
void motor_reset(void)
{
	PORT_MOTOR &= ~(1<<MOTOR1_RESET);	// Reset auf LO

	#if defined( PHB01_MOTOR2 )
		if (motor_cfg & MOTOR_CFG_USE_HBRIDGE2) { PORT_MOTOR &= ~(1<<MOTOR2_RESET); }
	#endif

	_delay_us(1);	// LO für 1µs

	PORT_MOTOR |= (1<<MOTOR1_RESET);	// Reset auf HI

	#if defined( PHB01_MOTOR2 )
		if (motor_cfg & MOTOR_CFG_USE_HBRIDGE2) { PORT_MOTOR |= (1<<MOTOR2_RESET); }
	#endif
}


//Error von MotorController checken (passt für phb01 und Pololu 24v20)
void checkMotorStatus()
{

	uint8_t error_alt = motorerror;
	motorerror = 0;
	char test[UART_MAXSTRLEN+1];

	if (motor_cfg & MOTOR_CFG_CHECK_ERR_HB1)
	{
		if (PIN_MOTOR & (1<<MOTOR1_FF1)) { motorerror |= 1; } 	// bit 0
		if (PIN_MOTOR & (1<<MOTOR1_FF2)) { motorerror |= 2; } 	// bit 1
	}


	#if defined( PHB01_MOTOR2 )
		if (motor_cfg & MOTOR_CFG_CHECK_ERR_HB2)
		{
			if (PIN_MOTOR & (1<<MOTOR2_FF1)) { motorerror |= 4; } 	// bit 2
			if (PIN_MOTOR & (1<<MOTOR2_FF2)) { motorerror |= 8; } 	// bit 3
		}
	#endif	// PHB01_MOTOR2


	if (motorerror > 0)	// bei Error: Speed auf 0 setzen -> stop (geschieht nicht sofort!!) passiert im speed.c
	{
		if (error_alt == 0)	// beim ersten Auftreten an Controller melden
		{
			memset(test, 0, UART_MAXSTRLEN+1);	// string leeren
			strlcpy_P(test, txtp_errmotor, 14);
			ltoa(motorerror, test+13, 10);
			strlcat_P(test, txtp_cmdend, UART_MAXSTRLEN+1); // will Länge des kompletten "test" buffers+0
			wlan_puts(test);
		}
	}
} // end checkMotorStatus()


//---------------------------------------------------------
// Warte-Funktion (Delay)
// Parameter: Wartezeit in 0...65535 ms
// Stimmt für alle Optimierungslevel + evt. Interruptausfuehrungszeiten
// mit for-Schleife da Fkt. delay_ms nicht für grosse Wartezeit geeignet
//---------------------------------------------------------
void warte_ms(unsigned int wartezeit)
{
	unsigned int i;
	for (i=0;i<wartezeit;i++)
		//wiederhole diese Schleife so oft wie Wartezeit angibt
	{
		_delay_ms(1);  //warte 1ms; Einbinden von delay.h noetig
	}
}

//--------------------------------------------------------------
// ersetzt CR und LF in einem String durch 0-bytes -> stringende
// (nicht besonders intelligent, aber einfach)
// Verbesserungsmöglichkeit: CR, LF aus dem string entfernen, alles andere vorher und nacher im string lassen
//--------------------------------------------------------------

void remcrlf(char *crlftext)
{
	int txtindex = 0;

	while(crlftext[txtindex] != 0)	// text text ab reply löschen (bis 0-byte)
	{
		if (crlftext[txtindex] == 10) 	{ crlftext[txtindex] = 0; }	// LF entfernen
		if (crlftext[txtindex] == 13) 	{ crlftext[txtindex] = 0; }	// CR entfernen
		txtindex++;
	}
}

// freie GPIOs initialisieren (als Ausgang setzen)
void init_gpios()
{
	// TODO: definierte ADC müssen auch noch ausgeschlossen werden (vorerst alle ADC ausgeschlossen)!!

	/*
	uint8_t portB_servo = 0;	// Masken für die einzelenen Ports
	uint8_t portD_servo = 0;
	uint8_t portE_servo = 0;
	uint8_t portG_servo = 0;

	for(uint8_t i = 0; i < servo_count; i++ )
	{
		if (servoPort[i] == 'B') { portB_servo += servoPin[i]; }
		else if (servoPort[i] == 'D') { portD_servo += servoPin[i]; }
		else if (servoPort[i] == 'E') { portE_servo += servoPin[i]; }
		else if (servoPort[i] == 'G') { portG_servo += servoPin[i]; }
	} */

	// uint8_t adc_mask = eeprom_getADCGPIO();	// bit = 1 bedeutet: wird für ADC (als Eingang) verwendendet - steht für GPIO nicht zur Verfügung

	// alles was auf 1 steht als Ausgänge setzen
	/*
	//DDRA |= GPIO_USABLE_PORT_A;	// keine freien GPIOs
	//DDRB |= (GPIO_USABLE_PORT_B & eeprom_getGPIO('B') & !portB_servo);
	DDRC |= GPIO_USABLE_PORT_C;	// Port C immer komplett als Ausgänge definieren
	DDRD |= (GPIO_USABLE_PORT_D & eeprom_getGPIO('D') & !portD_servo);
	DDRE |= (GPIO_USABLE_PORT_E & eeprom_getGPIO('E') & !portE_servo);
	//DDRF |= GPIO_USABLE_PORT_F;	// Port F (ADC) vorerst gar nicht, später ev. konfigurierbar machen
	DDRG |= (GPIO_USABLE_PORT_G & eeprom_getGPIO('G') & !portG_servo);
	*/

	DDRB |= filterGPIOMask('B', eeprom_getGPIO('B'));
	DDRC |= GPIO_USABLE_PORT_C;	// Port C immer komplett als Ausgänge definieren
	DDRD |= filterGPIOMask('D', eeprom_getGPIO('D'));
	DDRE |= filterGPIOMask('E', eeprom_getGPIO('E'));
	DDRG |= filterGPIOMask('G', eeprom_getGPIO('G'));

}


uint8_t getUsableGPIOs(char port)
{
	uint8_t mask = 0;

	switch (port)
	{
	case 'B':
		mask = GPIO_USABLE_PORT_B;
		break;

	case 'C':
		mask = GPIO_USABLE_PORT_C;
		break;

	case 'D':
		mask = GPIO_USABLE_PORT_D;
		break;

	case 'E':
		mask = GPIO_USABLE_PORT_E;
		break;

	case 'G':
		mask = GPIO_USABLE_PORT_G;
		break;
	}
	return mask;	// bei ungültigem Port wird (wie wenn nichts frei ist) 0 zurückgegeben
}

uint8_t filterGPIOMask(char port, uint8_t mask)	// usablemask und servomask für diesen Port prüfen, ob die GPIO-mask ok ist
{
	uint8_t servomask = 0;

	for(uint8_t i = 0; i < servo_count; i++ )	// Servopins für den port suchen
	{
		if (servoPort[i] == port) { servomask += (1<<servoPin[i]); }
	}

	mask = mask & getUsableGPIOs(port) & !servomask;	// pins müssen in usablemask und dürfen nicht in servomask vorhanden sein

	return mask;
}


uint8_t getGPIOs(char port)	// konfigurierte GPIOs (Outputs) für einen Port auslesen
{
	uint8_t gpiomask = 0;

	if (port == 'C') { gpiomask = 255;	} // Spezialfall Port C: immer ganzer Port GPIOs
	else { gpiomask = eeprom_getGPIO(port); }

	gpiomask = filterGPIOMask(port, gpiomask);

	return gpiomask;	// für ungültigen Port wird 0 zurückgegeben (das Problem muss dann über "usable" erkannt werden
}

// die Werte für die gesetzten Port-Pins auslesen
uint8_t getGPIOValues(char port)
{
	uint8_t values = 0;

	switch (port)
	{
	case 'B':
		values = PORTB;
		break;

	case 'C':
		values = PORTC;
		break;

	case 'D':
		values = PORTD;
		break;

	case 'E':
		values = PORTE;
		break;

	case 'G':
		values = PORTG;
		break;
	}

	values = getGPIOs(port) & values;	// nur die Werte für die verwendeten Pins liefern!!!

	return values;	// bei ungültigem Port wird 0 zurückgegeben
}


// Wert 0 oder 1 auf GPIO Port-Pin ausgeben (Gültigkeit Port/Pin muss vorher schon geprüft werden!)
// port: B / C / D / E / G
// pinnr: 0-7   pinval: 0 / 1
void setGPIOPin(char port, uint8_t pinnr, uint8_t pinval)
{
switch (port)
		{
		case 'B':
			if (pinval) { PORTB |= (1<<pinnr); }	// Pin auf 1 setzen
			else { PORTB &= ~(1<<pinnr); }			// Pin auf 0 setzen
			break;

		case 'C':
			if (pinval) { PORTC |= (1<<pinnr); }
			else { PORTC &= ~(1<<pinnr); }
			break;

		case 'D':
			if (pinval) { PORTD |= (1<<pinnr); }
			else { PORTD &= ~(1<<pinnr); }
			break;

		case 'E':
			if (pinval) { PORTE |= (1<<pinnr); }
			else { PORTE &= ~(1<<pinnr); }
			break;

		case 'G':
			if (pinval) { PORTG |= (1<<pinnr); }
			else { PORTG &= ~(1<<pinnr); }
			break;
		}
}

// Rückmeldetext für Hardware-Info-Anforderung (<hwget>) zusammenstellen
// Größe von txtbuffer muss sein: UART_MAXSTRLEN+1 (+1 wg. abschließender Null)
// txtbuffer muss leer sein (wird in befehl_auswerten() vorher schon gemacht)
// TODO: optimierung: es könnte hwi+ntypi+die ersten 2 Ports für's Senden zusammengefasst werden (max. 59 Zeichen ohne 0)
//       und die restlichen 3 Ports ebenfalls (max. 63 Zeiche ohne 0)
void sendHWinfo(char * txtbuffer)
{
	uint8_t txtsize = UART_MAXSTRLEN+1;
	//strlcat(txtbuffer, txtbuffer, txtsize);		// will länge des kompletten "test" buffers+0
	strlcat_P(txtbuffer, txtp_cmd_hwi, txtsize); 	//  für das UC02 Board mit "<hwi:01>" antworten
	strlcat_P(txtbuffer, txtp_cmd_ntypi, txtsize);	// Netzwerk-Typ (dummes WLAN-Modul)
	wlan_puts(txtbuffer);

	// GPIO-Port status für alle Ports B / C / D / E / G 	// TODO: später auch Soervo und ADC???
	char port[] = { 'B', 'C', 'D', 'E', 'G' };
	uint8_t gpiopins = 0;
	uint8_t usable_pins = 0;
	uint8_t values = 0;

	for (uint8_t i=0; i<5; i++)
	{
		gpiopins = getGPIOs(port[i]);
		usable_pins = getUsableGPIOs(port[i]);	// für ungültigen Port oder wenn nichts verwendet werden darf wird usable_pins = 0 zurückgegeben
		values = getGPIOValues(port[i]);
		sprintf_P(txtbuffer, txtp_cmd_gpioi, port[i], usable_pins, gpiopins, values);	//"<gpioi:%i:%i:%i:%i>" <gpioi:port:mögliche:verwendete:werte>    :char:byte-mask:byte-mask:byte-mask
		wlan_puts(txtbuffer);
	}
}

