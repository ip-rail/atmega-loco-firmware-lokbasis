/* lokbasis3b - für ATMega2561 im (Wiznet /) Raspi-Betrieb
 *
 * main.c
 *
 *	Version: 0.1
 *  Created on: 14.12.2014 - 06.05.2015
 *  Author: Michael Brunnbauer
 */


#ifndef F_CPU
#define F_CPU 16000000UL	// für 16MHz CPU-Takt
#endif


#ifndef EEMEM	//#include <avr/eeprom.h> ist zwar angegeben, EEMEM Definition funktioniert aber trotzdem nicht!!
#define EEMEM __attribute__((section(".eeprom")))	// deshalb hier nochmal
#endif

#include <avr/io.h>
#include <string.h>		// für "strcmp"
#include <stdlib.h>		// für "itoa"
#include <util/delay.h>	// für delay_ms()
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/twi.h>	// für I2C

#include "lokbasis_hwdef.h"		// Hardware-Definitionen für die verschiedenen Boards
#include "main.h"
#include "uart.h"
#include "commands.h"			// Befehle auswerten
#include "wlan.h"
#include "funktionen.h"			// allgemeine Funktionen (Hardware und andere)
#include "speed.h"				// Geschwindigkeit anpassen
#include "ledc.h"				// LED-Controller


//----------------------------------------------------------------------------------------------
volatile unsigned char state = 0;		// Status-Variable für Interrupts und Main-Schleife

unsigned int timer5_count = 0;			// zähler für timer5 (für isr, generelle Zeitintervalle)

char wlan_string[UART_MAXSTRLEN+1]="";  // globaler String zum Abspeichern des vom WLAN empfangen Strings in read_wlan


unsigned int speed = 0;					// globale variable für den Motor-Speed (reale Geschwindigkeit)
unsigned int speed_soll = 0;			// globale variable für den Motor-Speed - Vorgabe vom Controller
unsigned char richtung = RICHTUNG_VW;			// globlae variable für die Richtung (vorwärts = 1, rückwärts = 0)
unsigned char richtung_soll = RICHTUNG_VW;		// globlae variable für die Richtung - Vorgabe vom Controller (vorwärts = 1, rückwärts = 0)
unsigned char speedstep_korrektur = 0;			// Korrekturvariable, falls 8bit oder 9bit pwm statt 10bit verwendet wird
												// gibt an, um wieviele bit der speedwert geshiftet werden muss, damit er für die 10bit Verarbeitung passt (im timer3 interrupt)

unsigned char alivecount = 0;	// globlae Variable zum Zählen der Meldungen von der Gegenstelle
volatile char alivesecs = 0;	// globlae Variable die zählt, wie lange die Zählung geht -> man kann variable bestimmen, in welchem intervall geprüft wird

volatile unsigned char motor_reg = 1;	// Variable für Motor-Regelung ein/aus (auch für isr verwendet)
uint8_t motorerror = 0;					// Errorcode von Motorcontroller: 0 = kein Error



//struct devdata_t lokdata;			// Lokdata zur Laufzeit
//struct devdata_t EElokdata EEMEM;	// Lokdata EEPROM variable

//uint8_t eetest[4] EEMEM = { 2,4,6,8 };

// LokData im Flash
const char dev_swname[] PROGMEM = "lokbasis";     	// -> keine Änderung durch User -> flash
const char dev_swversion[] PROGMEM = "0.02";   		// -> keine Änderung durch User -> flash

//Strings im Flash für CMD-Rückmeldungen über WLAN
const char txtp_cmdend[] PROGMEM = ">";				// Befehlsende-Zeichen
const char txtp_errmotor[] PROGMEM = "<error:motor:";		// Motor-Error
const char txtp_sd[] PROGMEM = "<sd:";		// Speed-Rückmeldung




int main(void)
{
	char test[UART_MAXSTRLEN+1] = "";			// für string-Bearbeitungen (Rückmeldungen usw.)
	uint32_t loop_count = 0;					// Zähler für Hauptschleife
	uint32_t last_loop_count = 0;					// letztes gültiges Ergebnis des Zählers für Hauptschleife



	// ========================  Hardware Initialisierung  ========================================================

	init_uart(UART_NR_WLAN, UART_SETTING_WLAN);	// WLAN Daten Empfang aktivieren

	init_timer5(); 	// timer5 mit 244Hz (4ms)

	init_motorctrl();	// Ausgänge für Motorsteuerung, PWM

	#if defined( LEDC_PCA9622 ) || defined( LEDC_TLC59116 )
	ledcontrol_init(LEDC1);	// LED-Controller
	#endif	
	
	setbit(DDRD,PD6);	// Pin als Ausgang definieren - für Testsignal
	setbit(DDRD,PD7);	// Pin als Ausgang definieren - für Testsignal
	
	//init_gpios();		// frei verfügbare GPIOs als Ausgang definieren

	sei();	// Interrupts aufdrehen

	// ========================  Hardware Initialisierung abgeschlossen  ================================================


//-------------------------------------------------------------------------------------------------------------
//------                       H A U P T S C H L E I F E                                               --------
//-------------------------------------------------------------------------------------------------------------



	//uart0_putc('H');
	uart0_puts("<MC start>");


	setbit(PORTD,PD6);	// auf HI


	while (1)	// Hauptschleife (endlos)
	{
		loop_count++;	// Zähler für Hauptschleife

		PORT_TESTSIGNAL ^= (1<<TESTSIGNAL);	// Signale bei jedem Durchlauf togglen (oder 1x pro Sekunde weiter unten)
		//PORT_TESTSIGNAL ^= (1<<TESTSIGNAL2);


		if (alivesecs >= 3)	// //TODO lokdata // Prüfintervall = ALIVE_INTERVAL Sekunden! -> Lok stoppt nach ALIVE_INTERVAL sek herrenloser Fahrt
		{
			if (alivecount == 0)	// prüfen, ob in diesem Prüfintervall Meldungen der Gegenstelle einglangt sind
			{
				//alone = 1;


			}
				alivecount = 0;	// Zähler rücksetzen
				alivesecs = 0;
		}



		if (state & STATE_5X_PRO_SEK)
		{
			set_speed();		// setzt die aktuelle Geschwindigkeit und checkt, ob speed-Änderungen "gesoftet" werden müssen

			cli();
			state &= ~STATE_5X_PRO_SEK;	// state-flag zurücksetzen
			sei();
		}

		if (state & STATE_1X_PRO_SEK)		// Meldung an Server bei == 1 / Steuergerät (ca. 1x pro Sekunde)
		{
			//PORT_TESTSIGNAL ^= (1<<TESTSIGNAL);	// Signale 1x pro Sekunde togglen


			memset(test, 0, UART_MAXSTRLEN+1);	// string leeren
			strlcpy_P(test, txtp_sd, 5);	// Rückmeldung der Geschwindigkeit	//bei der Länge immer 0-Zeichen mitzählen!
			itoa(speed, test+4, 10);
			strlcat_P(test, txtp_cmdend, UART_MAXSTRLEN+1);	// will länge des kompletten "test" buffers+0
			uart0_puts(test);


			/*
			ltoa(loop_count, test, 10);	// testweise Rückmeldung des loopcount -> später an handy
			log_puts_P("loopcount = ");
			log_puts(test);
			log_puts_P("\r\n");
			*/

			// Spannungsmeldung Schiene, Akku und ev. Motoren

			last_loop_count = loop_count;	// ermittelten loopcount sichern
			loop_count = 0;	// loopcount sekündlich zurücksetzen -> ergibt loops/s
			cli();
			state &= ~STATE_1X_PRO_SEK;	// state-flag zurücksetzen
			sei();

		}


		if (uart0_available() > 0) { check_wlan_cmd(); }	// checken, ob WLAN-Daten vorhanden sind (wenn ja -> verarbeiten)


	}	// Ende Hauptschleife (endlos)


} // main ende




//-----------------------------------------------------------------------------------------
//----------------   Interrupts   ---------------------------------------------------------
//-----------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------------
// isr für timer 5 output compare match A interrupt: 244 Hz = 4ms
ISR(TIMER5_COMPA_vect) {

	timer5_count++;


	if (timer5_count % 48 == 0)	// 5x pro Sekunde. state: STATE_5X_PRO_SEK
	{
		state |= STATE_5X_PRO_SEK;	// state setzen
	}

	if (timer5_count == 244)	// 1x pro Sekunde
	{
		state |= STATE_1X_PRO_SEK;	// state setzen
		alivesecs++;
		timer5_count = 0;	// es wird nur bis 30 gezählt -> 1 Sekunde, dann wieder bei 0 anfangen
	}

}




