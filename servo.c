/*
 * servo.c
 *
 *  Created on: 19.06.2015
 *      Author: Michael Brunnbauer
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include "eedata.h"
#include "servo.h"

//#define DDR(x) (*(&x - 1))      // address of data direction register of port x  http://www.avrfreaks.net/forum/howto-using-variables-instead-predefined-port-macros


// Timer 0 (8bit) : zuständig für das 20ms Intervall
// Timer 4 (16bit): zuständig für die einzelnen Servo-Pulse

//Servo-Mode: 0

// verwendete IOs
// vorerst max. 6 Servo-Signale verwendbar
//servoPort[SERVOCOUNTMAX] = alt: { &PORTB, &PORTB, &PORTB, &PORTB, &PORTB, &PORTD } neu: { 'B', 'B', 'B', 'B', 'B', 'D' }
//servoPin[SERVOCOUNTMAX]  = { PB0, PB1, PB2, PB3, PB7, PD4 }	// Pin-Nummer


uint8_t servo_mode;	// 0: für jedes Servosignal wird ein GPIO verwendet (vordefinieren welche!). 1: alle Signale nur an 1 GPIO ausgeben (Port,Pin mit Index 0), an dem ein Kreiszähler-IC hängt
uint8_t servo_count;	// wieviele Servosignale ausgeben (max. 6-8?)
uint8_t servoPort[SERVOCOUNTMAX];	//Port und Pin der GPIOs, die für Servos verwendet werden (verwendet werden Index 0 bis SERVOCOUNTMAX-1)
uint8_t servoPin[SERVOCOUNTMAX];	//Port und Pin gehören je Index zusammen
uint8_t servo_sleep;		// grober Wert, wieviel von den 20ms Servo-Intervall gewartet werden muss
// diese Variablen ändern sich nach der Initialisierung nicht mehr, daher kein "volatile" nötig

//const uint8_t ServoOutput[SERVOCOUNT] = { 1<<PB1, 1<<PB2 };	// kein "volatile" nötig, da sich das nicht mehr ändert (-> const)

uint8_t servo_run;		// zum Stoppen der Signalgenerierung (wird im Timer4-Interrupt ausgelesen)
						// 0 = Signalgenerierung stoppen, 1 = weitermachen

// Werte für die Servoposition
// Gültige Werte laufen von 0 bis 2 * CENTER
// 0           ... ganz links
// CENTER      ... Mittelstellung
// 2 * CENTER  ... ganz rechts

volatile unsigned int servoValue[SERVOCOUNTMAX];

volatile unsigned int msCount;	// MilliSekunden Zähler (genau 1,024ms)
volatile uint8_t ServoId;		// welcher Servo ist dran


ISR (TIMER0_OVF_vect)      // wird alle 1,024ms ms aufgerufen, zuständig für's 20ms Intervall und Start der Servo Pulse
{

	msCount++;
	if(msCount >= servo_sleep)	// wenn die Wartezeit erreicht ist
	{
		// das nächste 20ms Intervall ist angebrochen
		ServoId = 0;
		//SERVO_PORT |= ServoOutput[ServoId];	// Puls für ersten Servo starten	// alt
		// http://openbook.rheinwerk-verlag.de/c_von_a_bis_z/012_c_zeiger_007.htm#mjb99637a42fd3decdfe07fe3416407be8
		//(*(uint8_t *)(servoPort)) |= servoPin[0];	// Puls für ersten Servo starten //laut ANSI-C-Standard wird der Array-Name immer als Zeiger auf das erste Array-Element angesehen

		switch(servoPort[0])
		{
			case 'B':
				PORTB |= (1<<servoPin[0]);
				break;

			case 'D':
				PORTD |= (1<<servoPin[0]);
				break;

			case 'E':
				PORTE |= (1<<servoPin[0]);
				break;

			case 'G':
				PORTG |= (1<<servoPin[0]);
				break;
		}

		OCR4A = MILLISEC_BASE + servoValue[ServoId];	// Impulsdauer für den Servo setzen
		TCCR4B |= (1 << CS40);	//timer4 für ersten Servo starten, prescaler=1

		TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00)); // timer0 stoppen
		TCNT0 = 0;	// timer0 auf 0 setzen
		msCount = 0;
	}

}

ISR (TIMER4_COMPA_vect)	// beendet den jweils aktuellen Servo-Puls und kümmert sich um die nachfolgende Aktion (weiter Puls oder Ende)
{
	uint8_t sid = ServoId;	// volatile Variable in ein Register laden (dh. wenn's der Compiler macht)
	uint8_t sid_out = ServoId;	//für Ausgabe, muss bei servo_mode=1 immer 0 sein!

	if(servo_mode) { sid_out = 0; }	// alle Servosignale am selben (ersten) GPIO ausgeben (für nachgeschalteten Kreiszähler-IC)

	// SERVO_PORT &= ~ServoOutput[ServoId];  // den Puls des aktuellen Servos beenden // alt
	//(*(uint8_t *)(servoPort + sid_out)) &= ~servoPin[sid_out];  // den Puls des aktuellen Servos beenden

	switch(servoPort[sid_out])
	{
		case 'B':
			PORTB &= ~(1<<servoPin[sid_out]);
			break;

		case 'D':
			PORTD &= ~(1<<servoPin[sid_out]);
			break;

		case 'E':
			PORTE &= ~(1<<servoPin[sid_out]);
			break;

		case 'G':
			PORTG &= ~(1<<servoPin[sid_out]);
			break;
	}

	if( ++sid >= servo_count ) // alle Servos haben ihren aktuellen Impuls bekommen?
	{
		if (servo_run)	// nur machen, wenn weiter Servosignale generiert werden sollen
		{
			// timer0 starten, der sich um den Rest der 20ms kümmert (in dem keine Servo-Impulse ausgegeben werden)
			TCCR0B |=  (1 << CS01) | ( 1 << CS00 );    	// Prescaler: 64 (startet den timer!)
		}

		// dann timer4 stoppen und alles für einen neuen Durchlauf vorbereiten
		sid = 0;

		// TCCR4B CS42, CS41, CS40: clock select
		// 000: timer stopped
		TCCR4B &= ~((1 << CS42) | (1 << CS41) | (1 << CS40)); // timer4 stoppen
		TCNT4 = 0;	//timer4 auf 0 setzen



	}
	else
	{
		// SERVO_PORT |= ServoOutput[ServoId];	// die Puls für den neuen Servo auf HIGH	// alt
		//(*(uint8_t *)(servoPort + sid_out)) |= servoPin[sid_out];	// die Puls für den neuen Servo auf HIGH

		switch(servoPort[sid_out])
		{
			case 'B':
				PORTB |= (1<<servoPin[sid_out]);
				break;

			case 'D':
				PORTD |= (1<<servoPin[sid_out]);
				break;

			case 'E':
				PORTE |= (1<<servoPin[sid_out]);
				break;

			case 'G':
				PORTG |= (1<<servoPin[sid_out]);
				break;
		}

	}

	OCR4A = MILLISEC_BASE + servoValue[sid]; // Impulsdauer für den nächsten Servo setzen
	ServoId = sid;	// ServoId zurückspeichern
}


void initServo()
{
	uint8_t i;

	// folgende Werte sind nicht mehr konstant und müssen daher berechnet werden:
	//#define SERVOCOUNT      2 <- aus eeprom-daten lesen
	//#define SERVO_DDR      DDRB	- wird durch servoPort[] ersetzt
	//#define SERVO_PORT     PORTB  - wird durch servoPin[] ersetzt

	servo_run = 1;	//	Variable zum Stoppen der Servo-Signalgenereierung
	// gestartet wird mit dem Starten von timer4, ist in initServo() aber auskommentiert


	// Servo Konfiguration aus EEPROM auslesen und globale Variablen befüllen!!!

	eeprom_getServoCount();
	if (servo_count == 0) { return; }	// wenn keine Servos definiert sind, keine GPIOs initialisieren!!

	eeprom_getServoMode();
	eeprom_getServoGPIO();

	servo_sleep = 20 - (servo_count * 2) - 1;	//#define SERVOSLEEP     20 - (SERVOCOUNT * 2) - 1	// grob, muss nicht genau sein
	ServoId = 0;

	//hier muss DDRx gesetzt werden, nicht Portx !!!!! DDR-Adresse ist Port-Adresse - 1
	if(servo_mode) // servo_mode == 1: nur ersten Servo-GPIO verwenden
	{
		//DDR(servoPort[0]) |= servoPin[0];

		switch(servoPort[0])
		{
		case 'B':
			DDRB |= (1<<servoPin[0]);
			break;

		case 'D':
			DDRD |= (1<<servoPin[0]);
			break;

		case 'E':
			DDRE |= (1<<servoPin[0]);
			break;

		case 'G':
			DDRG |= (1<<servoPin[0]);
			break;
		}
	}
	else	 // alle Servoleitungen auf Ausgang stellen
	{
		for( i = 0; i < servo_count; i++ )
		{
			//DDR(servoPort[i]) |= servoPin[i];

			switch(servoPort[i])
			{
			case 'B':
				DDRB |= (1<<servoPin[i]);
				break;

			case 'D':
				DDRD |= (1<<servoPin[i]);
				break;

			case 'E':
				DDRE |= (1<<servoPin[i]);
				break;

			case 'G':
				DDRG |= (1<<servoPin[i]);
				break;
			}
		}
	}

	for( i = 0; i < servo_count; i++ ) { servoValue[i] = CENTER; }  // Alle Servos in Mittelstellung


	// Timer 0 Standardmode, nur zählen, daher (WGM02:0 = 0) -> passt Startwerte passen bereits
	///TCCR0B |=  (1 << CS01) | ( 1 << CS00 );    	// Prescaler: 64 (startet den timer!)
	// bei 16MHz, 64 Prescaler wird alle 1,024ms ein ein Overflow Interrupt ausgeloest
	// so kann die Zeit auf 20ms gezählt und aufgefüllt werden
	TIMSK0 = ( 1 << TOIE0 );   // Overflow Interrupt einschalten


	// Timer4 auf CTC Modus konfigurieren
	OCR4A = MILLISEC_BASE + servoValue[0];
	TIMSK4 |= (1<<OCIE4A);

	// TCCR4B WGM43 0
	// TCCR4B WGM42 1
	// TCCR4A WGM41 0
	// TCCR4A WGM40 0

	// TCCR4A ist sowieso vom Start alles auf 0 (OC1A,OC1B normal port operation, kein automatischer timer output)

	// TCCR4B CS42, CS41, CS40: clock select
	// 000 timer stopped
	// 001 prescaler =1 **** verwenden!!!
	// 010 prescaler =8
	// 011 prescaler =64
	// 100 prescaler =256
	// 101 prescaler =1024

	TCCR4B |= (1<<WGM42);		// CTC mode


	// alles ist bereit, Signalgenerierung startet aber erst mit dem Starten von timer 4!
	// Stoppen der Signalgenerierung wäre mit: servo_run=0;
	// TCCR4B |= (1 << CS40);	//start timer4, prescaler=1	// nicht hier starten!
}

void Servo_stop()	// Stoppen der Signalgenerierung für die Servos
{
	servo_run=0;
}

void Servo_start()	// Signalgenerierung für Servos starten (initServo() muss natürlich schon gelaufen sein)
{
	TCCR4B |= (1 << CS40);	//start timer4, prescaler=1
}
