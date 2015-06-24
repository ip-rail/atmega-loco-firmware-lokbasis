/*
 * adc.c
 *
 *  Created on: 24.06.2015
 *      Author: Michael Brunnbauer
 */


#include <avr/io.h>
//#include <string.h>		// für "strcmp"
//#include <stdlib.h>		// für "itoa"
//#include <util/delay.h>	// für delay_ms()
#include <avr/interrupt.h>
//#include <avr/pgmspace.h>

#include "lokbasis_hwdef.h"
#include "eedata.h"
#include "main.h"
#include "adc.h"
//#include "servo.h"
//#include "uart.h"
//#include "funktionen.h"


// ADC initialisieren
void init_adc()
{
	// https://www.mikrocontroller.net/articles/AVR-GCC-Tutorial/Analoge_Ein-_und_Ausgabe#ADC_.28Analog_Digital_Converter.29
	clearbit(PRR0,PRADC);	// Power Reduction bit disabled
	/*clearbit(DDRF,0);		// Port F  (ADC0-7)  auf Eingang (0) schalten
	clearbit(DDRF,1);
	clearbit(DDRF,2);
	clearbit(DDRF,3);
	clearbit(DDRF,4);
	clearbit(DDRF,5);
	clearbit(DDRF,6);
	clearbit(DDRF,7); */

	//TODO: entweder anhand der ADC-Maske checken, welche ADC channels gelesen werden müssen (und die unnötigen überspringen) oder es darf keine Lücken
	// geben und es muss aus der Maske die Anzahl der verwendeten Channels ermittelt werden -> egal welche Methode -> in der ISR berücksichtigen

	adc_mask = eeprom_getADCGPIO();	// gespeicherte ADC-Konfiguration aus EEPROM laden
	if (adc_mask == 0) { return; }	// nichts zu tun!

	DDRF &= ~adc_mask;	// gesetzte ADC-Kanäle auf Eingang (0) schalten

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
	adcchannel = 0;	// ADC0

	// den ersten verwendeten Kanal suchen
	while(!((1<<adcchannel) & adc_mask)) {
		adcchannel++;
		if (adcchannel > 7) { break; }
	}


	ADMUX = (ADMUX & ~(0x1F)) | (adcchannel & 0x1F);
	ADCSRB &= ~((1<< MUX5));	//oberstes bit MUX5 (in anderem Register) ist für single channel ADC0-7 immer 0! //MUX5 =3 (bit 3)

	setbit(ADCSRA,ADEN);	// ADC enable

	//setbit(ADCSRA,ADIE);	// ADC interrupt enable
	//setbit(ADCSRA,ADSC);	// start (each) ADC conversion
	//erste Messung immer ignorieren


}



// ADC Interrupt routine

ISR(ADC_vect) {

	unsigned int val;

	//TODO: ACHTUNG: alles noch alt: muss noch geändert werden (ist aus "lokbasis1 - Kopie v0.7")
	// var:
	// volatile uint8_t adcchannel = 0;			// aktueller ADC channel (0-7)
	// volatile uint8_t adcreadcount = 0;		// counter für Lesevorgänge pro ADC channel
	// volatile unsigned int adcvalue[adcchannel][adcreadcount-1] // 8 cannels, 4 Werte werden gelesen (der erste wird ignoriert!)

	// auslesen und in richtiger Variable speichern
	// 3 oder 5 werte lesen, den 1. verwerfen
	// danach auf nächsten port umstellen
	// dann in der Hauptschleife den Wert mitteln

	// alt: nach der Methode kracht es!!! Reboots osw!! solbad der Interrupt deaktiviert ist, passt alles!!



	val = ADC;	//Wert muss immer gelesen werden

	if (adcreadcount > 0)
	{
		adcvalue[adcchannel][adcreadcount-1] = val;
	}

	adcreadcount++;

	if (adcreadcount > 4)
	{
		adcreadcount = 0;
		adcchannel++;
		if (adcchannel > 7) { adcchannel = 0; }

		while(!((1<<adcchannel) & adc_mask))// nicht verwendete Kanäle überspringen
		{
			adcchannel++;
			if (adcchannel > 7)
			{
				adcchannel = 0;
				break;
			}
		}

		ADMUX = (ADMUX & ~(0x1F)) | (adcchannel & 0x1F);	// nächsten Channel einstellen
	}



	setbit(ADCSRA,ADSC);	// start next conversion


}


