/*
 * servo.c
 *
 *  Created on: 19.06.2015
 *      Author: Michael Brunnbauer
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>		// für "sprintf"!!! TODO: nur für Tests: wieder weg

#include "eedata.h"
#include "servo.h"
#include "wlan.h"
#include "main.h"

// Timer 0 (8bit) : zuständig für das 20ms Intervall
// Timer 4 (16bit): zuständig für die einzelnen Servo-Pulse

//Servo-Mode: 0
// vorerst max. 6 Servo-Signale verwendbar

uint8_t servo_mode;	// 0: für jedes Servosignal wird ein GPIO verwendet (vordefinieren welche!). 1: alle Signale nur an 1 GPIO ausgeben (Port,Pin mit Index 0), an dem ein Kreiszähler-IC hängt
uint8_t servo_count;	// wieviele Servosignale ausgeben (max. 6-8?)
uint8_t servoPort[SERVOCOUNTMAX];	//Port und Pin der GPIOs, die für Servos verwendet werden (verwendet werden Index 0 bis SERVOCOUNTMAX-1)
uint8_t servoPin[SERVOCOUNTMAX];
uint8_t servoOutput[SERVOCOUNTMAX];	//Port, Pin und Output gehören je Index zusammen (Output = << geshifteter Pin)

uint8_t servo_sleep;		// grober Wert, wieviel von den 20ms Servo-Intervall gewartet werden muss
// diese Variablen ändern sich nach der Initialisierung nicht mehr, daher kein "volatile" nötig

volatile uint8_t servo_run;		// zum Stoppen der Signalgenerierung (wird im Timer4-Interrupt ausgelesen)
						// 0 = Signalgenerierung stoppen, 1 = weitermachen

volatile uint8_t servo_error;	// error bei servo IO

// Werte für die Servoposition
// Gültige Werte laufen von 0 bis 2 * CENTER
// 0           ... ganz links
// CENTER      ... Mittelstellung
// 2 * CENTER  ... ganz rechts

volatile unsigned int servoValue[SERVOCOUNTMAX];

volatile unsigned int msCount;	// MilliSekunden Zähler (genau 1,024ms)
volatile uint8_t ServoId;		// welcher Servo ist dran


ISR (TIMER0_OVF_vect)      // wird alle 1,024ms ms aufgerufen, zuständig für's 20ms Intervall und Start Puls für ersten Servo
{

	msCount++;
	if(msCount >= servo_sleep)	// wenn die Wartezeit erreicht ist
	{
		// das nächste 20ms Intervall ist angebrochen -> also den ersten Servo-Pin auf High setzen
		ServoId = 0;

		if (servoPort[0] == 'B') { PORTB |= servoOutput[0]; }
		else if (servoPort[ServoId] == 'D') { PORTD |= servoOutput[0]; }
		else if (servoPort[ServoId] == 'E') { PORTE |= servoOutput[0]; }
		else if (servoPort[ServoId] == 'G') { PORTG |= servoOutput[0]; }

		OCR4A = MILLISEC_BASE + servoValue[ServoId];	// Impulsdauer für den Servo setzen
		TCCR4B |= (1 << CS40);	//timer4 für ersten Servo starten, prescaler=1

		TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00)); // timer0 stoppen
		TCNT0 = 0;	// timer0 auf 0 setzen
		msCount = 0;
	}

}

ISR (TIMER4_COMPA_vect)	// beendet den jweils aktuellen Servo-Puls und kümmert sich um die nachfolgende Aktion (weiter Puls oder Ende)
{
	// servo_mode == 1 wird noch nicht unterstützt!
	//uint8_t sid_out = ServoId;	//für Ausgabe, muss bei servo_mode=1 immer 0 sein!
	//if(servo_mode) { sid_out = 0; }	// alle Servosignale am selben (ersten) GPIO ausgeben (für nachgeschalteten Kreiszähler-IC)

	//servopuls beenden
	if (servoPort[ServoId] == 'B') { PORTB &= ~servoOutput[ServoId]; }
	else if (servoPort[ServoId] == 'D') { PORTD &= ~servoOutput[ServoId]; }
	else if (servoPort[ServoId] == 'E') { PORTE &= ~servoOutput[ServoId]; }
	else if (servoPort[ServoId] == 'G') { PORTG &= ~servoOutput[ServoId]; }

	ServoId++;	// ServoID erhöhen, checken: haben alle Servos ihren aktuellen Impuls bekommen?

	if(ServoId >= servo_count)
	{
		//if (servo_run)
		//{
			TCCR0B |=  (1 << CS01) | ( 1 << CS00 );    	// Prescaler: 64 (startet den timer!)
		//}

		// dann timer4 stoppen und alles für einen neuen Durchlauf vorbereiten
		ServoId = 0;

		// TCCR4B CS42, CS41, CS40: clock select
		// 000: timer stopped
		TCCR4B &= ~((1 << CS42) | (1 << CS41) | (1 << CS40)); // timer4 stoppen
		TCNT4 = 0;	//timer4 auf 0 setzen
	}
	else // nächsten Servo-Impuls im aktuellen 20ms Intervall starten
	{
		// Puls für den neuen Servo auf HIGH // todo: sid_out statt ServoId
		if (servoPort[ServoId] == 'B') { PORTB |= servoOutput[ServoId]; }
		else if (servoPort[ServoId] == 'D') { PORTD |= servoOutput[ServoId]; }
		else if (servoPort[ServoId] == 'E') { PORTE |= servoOutput[ServoId]; }
		else if (servoPort[ServoId] == 'G') { PORTG |= servoOutput[ServoId]; }
	}

	OCR4A = MILLISEC_BASE + servoValue[ServoId];
}


void initServo()
{

	// folgende Werte sind nicht mehr konstant und müssen daher berechnet werden:
	//#define SERVOCOUNT      2 <- aus eeprom-daten lesen
	//#define SERVO_PORT     PORTB  - wird durch servoPort[] ersetzt

	servo_run = 1;	//	Variable zum Stoppen der Servo-Signalgenereierung
	// gestartet wird mit dem Starten von timer4, ist in initServo() aber auskommentiert
	servo_error = 0;

	// Servo Konfiguration aus EEPROM auslesen und globale Variablen befüllen!!!

	eeprom_getServoCount();
	if (servo_count == 0) { return; }	// wenn keine Servos definiert sind, keine GPIOs initialisieren!!

	eeprom_getServoMode();

	eeprom_getServoGPIO();
	for(int i = 0; i < servo_count; i++) { servoOutput[i] = (1<<(servoPin[i])); }	// servoOutput generieren

	servo_sleep = 20 - (servo_count * 2) - 1;	//#define SERVOSLEEP     20 - (SERVOCOUNT * 2) - 1	// grob, muss nicht genau sein
	ServoId = 0;

	// alle Servoleitungen auf Ausgang stellen
	for(int i = 0; i < servo_count; i++ )
	{
		if (servoPort[i] == 'B') 	  { DDRB |= servoOutput[i]; }
		else if (servoPort[i] == 'D') { DDRD |= servoOutput[i]; }
		else if (servoPort[i] == 'E') { DDRE |= servoOutput[i]; }
		else if (servoPort[i] == 'G') { DDRG |= servoOutput[i]; }
	}

	for(int i = 0; i < servo_count; i++ ) { servoValue[i] = CENTER; }  // Alle Servos in Mittelstellung


	// Timer 0 Standardmode, nur zählen, daher (WGM02:0 = 0) -> passt Startwerte passen bereits
	///TCCR0B |=  (1 << CS01) | ( 1 << CS00 );    	// Prescaler: 64 (startet den timer!)
	// bei 16MHz, 64 Prescaler wird alle 1,024ms ein ein Overflow Interrupt ausgeloest
	// so kann die Zeit auf 20ms gezählt und aufgefüllt werden
	TIMSK0 = ( 1 << TOIE0 );   // Overflow Interrupt einschalten


	// Timer4 auf CTC Modus konfigurieren

	// TCCR4B WGM43 0
	// TCCR4B WGM42 1
	// TCCR4A WGM41 0
	// TCCR4A WGM40 0

	TCCR4B |= (1<<WGM42);		// CTC mode

	// TCCR4A ist sowieso vom Start alles auf 0 (OC1A,OC1B normal port operation, kein automatischer timer output)

	TCNT4 = 0;	//timer4 auf 0 setzen

	OCR4A = MILLISEC_BASE + servoValue[0];

	TIMSK4 |= (1<<OCIE4A);

	// TCCR4B CS42, CS41, CS40: clock select
	// 000 timer stopped
	// 001 prescaler =1 **** verwenden!!!
	// 010 prescaler =8
	// 011 prescaler =64
	// 100 prescaler =256
	// 101 prescaler =1024

	// alles ist bereit, Signalgenerierung startet aber erst mit dem Starten von timer4!
	// Stoppen der Signalgenerierung wäre mit: servo_run=0;
	//TCCR4B |= (1<<CS40);	//start timer4, prescaler=1	// nicht hier starten! // TODO: für Test aktiviert
}

void Servo_stop()	// Stoppen der Signalgenerierung für die Servos
{
	servo_run=0;
}

void Servo_start()	// Signalgenerierung für Servos starten (initServo() muss natürlich schon gelaufen sein)
{
	TCCR4B |= (1 << CS40);	//start timer4, prescaler=1 // TODO: für Test deaktiviert
	// TODO: passt nicht ganz: Impuls für ersten Servo wird von der Timer0-ISR gestartet
}
