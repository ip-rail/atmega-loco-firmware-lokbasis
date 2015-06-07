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
#include <stdlib.h>		// für "itoa"
#include <util/delay.h>	// für delay_ms()
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "lokbasis_hwdef.h"
#include "main.h"
#include "uart.h"
#include "funktionen.h"



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
		setbit(UCSR0B,RXCIE0);	// RX Interrupt einschalten
		clearbit(UCSR0B,UCSZ02);// UCSZ02 -> 0 für 8bit

		// Asynchron, 8N1
		clearbit(UCSR0C,UMSEL01);	// asynchrone Übertragung
		clearbit(UCSR0C,UMSEL00);	// asynchrone Übertragung
		setbit(UCSR0C,UCSZ00);		// 8bit
		setbit(UCSR0C,UCSZ01);
		clearbit(UCSR0C,UPM00);		// no parity
		clearbit(UCSR0C,UPM01);
		clearbit(UCSR0C,USBS0);		// 1 stopbit


		break;

#if defined( ATMEGA_USART1 )
	case 1:

		/* TODO: nur test
		UART1_TxHead = 0;
		UART1_TxTail = 0;
		UART1_RxHead = 0;
		UART1_RxTail = 0;

		UBRR1H = (unsigned char) (uartsetting >> 8);	// Baudrate einstellen: H muss vor L geschrieben werden!!
		UBRR1L = (unsigned char) (uartsetting);
		UCSR1B = (1<<RXEN1) | (1<<TXEN1);	// senden & empfangen einschalten
		UCSR1C = (1<<UCSZ10) | (1<<UCSZ11);	// asynchron 8N1
		UCSR1B |= (1<<RXCIE1);	// RX Interrupt einschalten
		*/
		break;
#endif // ATMEGA_USART1

	default:
		break;
	}
}


// Motorsteuerung initialisieren

void init_motorctrl(void)
{

#if defined( HW_UC02 )	//WLANcroc H-Brücke

	#if defined( PHB01_MOTOR1 )
		DDR_MOTOR |= (1<<MOTOR1_DIR) | (1<<MOTOR1_RESET);	// als Ausgänge setzen - nur 1 H-Brücke
		DDR_MOTOR &= ~((1<<MOTOR1_FF1) | (1<<MOTOR1_FF2));	// Fehlerpins als Eingänge setzen

		//PORT_MOTOR |= (1<<MOTOR1_DIR);	// Richtung auf 1 setzen // je nach Motorcontroller
		PORT_MOTOR &= ~(1<<MOTOR1_DIR);   	// Motor-Richtung auf 0 // bei diesem Controller: vorwärts
		PORT_MOTOR |= (1<<MOTOR1_RESET);	// Reset auf HI

		MOTOR1_OCR = 0;

	#endif

	#if defined( PHB01_MOTOR2 )
		DDR_MOTOR |= (1<<MOTOR2_DIR) | (1<<MOTOR2_RESET);	// als Ausgänge setzen
		DDR_MOTOR &= ~((1<<MOTOR2_FF1)  | (1<<MOTOR2_FF2));	// Fehlerpins als Eingänge setzen

		//PORT_MOTOR |= (1<<MOTOR2_DIR);	// Richtung auf 1 setzen // je nach Motorcontroller
		PORT_MOTOR &= ~(1<<MOTOR2_DIR);   	// Motor-Richtung auf 0 // bei diesem Controller: vorwärts
		PORT_MOTOR |= (1<<MOTOR2_RESET);	// Reset auf HI

		MOTOR2_OCR = 0;
	#endif


		// Timer 1 für Motor-PWM initialisieren --------------------------------------
		init_pwm(8);	// Mode 8: 15656Hz 9bit prescaler 1  - Timer 1 für Motor-PWM initialisieren

#endif
}



void init_pwm(char freq_pwm)	// Timer 1 für Motor-PWM initialisieren
{

	speedstep_korrektur = 0;	// gibt an, ob speed *2 oder *4 genommen werden muss (8/9bit), globale varaible für speed-stellung
	//clearbit(TIMSK1,OCIE1A);	// Compare Match Interrupt abdrehen (wird für Motor-Messung verwendet) - war für motor-mess-test

	if ((freq_pwm < 1) | (freq_pwm > 9)) { freq_pwm = 1; }

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


	// DDR_MOTOR_PWM |= (1<<MOTOR1_PWM) | (1<<MOTOR2_PWM);	// als Ausgänge setzen
	DDR_MOTOR_PWM |= (1<<MOTOR1_PWM);	// als Ausgänge setzen - nur 1 H-Brücke

	#if defined( PHB01_MOTOR1 )
		DDR_MOTOR_PWM |= (1<<MOTOR1_PWM);	// als Ausgang setzen - 1. H-Brücke
		MOTOR1_OCR = 0;
	#endif

	#if defined( PHB01_MOTOR2 )
		DDR_MOTOR_PWM |= (1<<MOTOR2_PWM);	// als Ausgang setzen - 2. H-Brücke
		MOTOR2_OCR = 0;
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

		default:	// 122Hz 8bit:256, 10bit:64*
			setbit(TCCR1A,WGM11);
			setbit(TCCR1A,WGM10);
			clearbit(TCCR1B,CS12);		// prescaler = 64
			setbit(TCCR1B,CS11);		// prescaler = 64
			setbit(TCCR1B,CS10);		// prescaler = 64
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
}

// Motorcontroller PHB01 aus Schlafmodus aufwecken (dauert ca. 3ms)
void motor_wakeup(void)
{
	PORT_MOTOR |= (1<<MOTOR1_RESET);	// Reset auf HI
	//warte_ms(4);	// TODO: Wartezeit sinnvoll?
}

// Motorcontroller PHB01 Reset zum Fehlerquitieren (wie lange auf LOW: >0,1µs <3,5µs)
void motor_reset(void)
{
	PORT_MOTOR &= ~(1<<MOTOR1_RESET);	// Reset auf LO
	_delay_us(1);	// LO für 1µs
	PORT_MOTOR |= (1<<MOTOR1_RESET);	// Reset auf HI
}


//Error von MotorController checken (passt für phb01 und Pololu 24v20)
void checkMotorStatus()
{

	uint8_t error_alt = motorerror;
	motorerror = 0;
	char test[UART_MAXSTRLEN+1];

	if (PIN_MOTOR & (1<<MOTOR1_FF1)) { motorerror |= 1; } 		else { motorerror &= ~1; }	// bit 0
	if (PIN_MOTOR & (1<<MOTOR1_FF2)) { motorerror |= (1<<1); } 	else { motorerror &= ~2; }	// bit 1

	#if defined( PHB01_MOTOR2 )

			if (PIN_MOTOR & (1<<MOTOR2_FF1)) { motorerror |= (1<<2); } 	else { motorerror &= ~4; }	// bit 2
			if (PIN_MOTOR & (1<<MOTOR2_FF2)) { motorerror |= (1<<3); } 	else { motorerror &= ~8; }	// bit 3
	#endif	// PHB01_MOTOR2


	if (motorerror > 0)	// bei Error: Speed auf 0 setzen -> stop (geschieht nicht sofort!!)
	{
		if (error_alt == 0)	// beim ersten Auftreten an Controller melden
		{
			memset(test, 0, UART_MAXSTRLEN+1);	// string leeren
			strlcpy_P(test, txtp_errmotor, 14);
			ltoa(motorerror, test+13, 10);
			strlcat_P(test, txtp_cmdend, UART_MAXSTRLEN+1); // will Länge des kompletten "test" buffers+0
			uart0_puts(test);
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
	// alles was auf 1 steht als Ausgänge setzen
	DDRA |= GPIO_USABLE_PORT_A;
	DDRB |= GPIO_USABLE_PORT_B;
	DDRC |= GPIO_USABLE_PORT_C;
	DDRD |= GPIO_USABLE_PORT_D;
	DDRE |= GPIO_USABLE_PORT_E;
	DDRF |= GPIO_USABLE_PORT_F;
	DDRG |= GPIO_USABLE_PORT_G;
}


// ADC initialisieren
void init_adc()
{
	// https://www.mikrocontroller.net/articles/AVR-GCC-Tutorial/Analoge_Ein-_und_Ausgabe#ADC_.28Analog_Digital_Converter.29
	clearbit(PRR0,PRADC);	// Power Reduction bit disabled
	clearbit(DDRF,0);		// Port F  (ADC0-7)  auf Eingang (0) schalten
	clearbit(DDRF,1);
	clearbit(DDRF,2);
	clearbit(DDRF,3);
	clearbit(DDRF,4);
	clearbit(DDRF,5);
	clearbit(DDRF,6);
	clearbit(DDRF,7);

	//setbit(DIDR0,ADC0D);	// digital input buffer disabled für ADC0 pin (reduces power consumption, besser für ADC)
	DIDR0 = 0xff;			// gleich für den ganzen Port F setzen (ADC0D - ADC7D)

	clearbit(ADCSRA,ADATE);	// nicht: auto trigger enable
	clearbit(ADMUX,ADLAR);	// right-justified
	setbit(ADCSRA,ADPS2);	// prescaler = 128 (die langsamste Variante)
	setbit(ADCSRA,ADPS1);	// prescaler = 128
	setbit(ADCSRA,ADPS0);	// prescaler = 128

	clearbit(ADMUX,REFS1);	// AVCC (5V) Referenz
	setbit(ADMUX,REFS0);	// AVCC (5V) Referenz



	// Kanal wählen Beispiel:
	uint8_t channel = 0;	// ADC0
	ADMUX = (ADMUX & ~(0x1F)) | (channel & 0x1F);
	clearbit(ADCSRB,MUX5);	//oberstes bit (in anderem Register) ist für single channel ADC0-7 immer 0! //MUX5 =3 (bit 3)

	ADCSRB &= ~((1<< MUX5));

	setbit(ADCSRA,ADEN);	// ADC enable

	//setbit(ADCSRA,ADIE);	// ADC interrupt enable
	//setbit(ADCSRA,ADSC);	// start (each) ADC conversion
	//erste Messung immer ignorieren


}



