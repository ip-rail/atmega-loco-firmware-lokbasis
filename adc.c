/*
 * adc.c
 *
 *  Created on: 24.06.2015
 *      Author: Michael Brunnbauer
 */


#include <stdlib.h>		// für "itoa"
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "lokbasis_hwdef.h"
#include "eedata.h"
#include "main.h"
#include "wlan.h"
#include "uart.h"
#include "adc.h"


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
	clearbit(DDRF,0);		// Port F  (ADC0-7)  auf Eingang (0) schalten	// TODO: test , nur zur sicherheit nochmal, wieder entfernen!


	//setbit(DIDR0,ADC0D);	// digital input buffer disabled für ADC0 pin (reduces power consumption, besser für ADC)
	DIDR0 = 0xff;			// gleich für den ganzen Port F setzen (ADC0D - ADC7D)

	clearbit(ADCSRA,ADATE);	// nicht: auto trigger enable
	clearbit(ADMUX,ADLAR);	// right-justified
	setbit(ADCSRA,ADPS2);	// prescaler = 128 (die langsamste Variante)
	setbit(ADCSRA,ADPS1);	// prescaler = 128
	setbit(ADCSRA,ADPS0);	// prescaler = 128

	clearbit(ADMUX,REFS1);	// AVCC (5V) Referenz
	setbit(ADMUX,REFS0);	// AVCC (5V) Referenz

	uint8_t i;
	for( i = 0; i < 8; i++ )
	{
		adcvalue[i] = 0xffff;	// ffff = kein gültiger Wert vorhanden
	}


	adcreadcount = 0;
	// Kanal wählen Beispiel:
	adcchannel = 0;	// ADC0

	// den ersten verwendeten Kanal suchen
	while(!((1<<adcchannel) & adc_mask)) {
		adcchannel++;
		if (adcchannel > 7)
		{
			adcchannel = 0;	// damit kein Unglück geschieht (adcchannel darf nicht > 7 werden)
			break;
		}
	}

	// ADMUX = (ADMUX & ~(0x1F)) | (1<<adcchannel & 0x1F);	// Kanal auswählen - funkt so nicht!!!!!


	// es müssen also nur MUX0 bis MUX2 gesetzt werden
	// MUX3 und MUX4 sind immer 0 für single channel ADC0-7

	for (i = 0; i < 3; i++ )
	{
		if (adcchannel & 1<<i) { setbit(ADMUX,i); }
		else { clearbit(ADMUX,i); }
	}

	clearbit(ADMUX,MUX3);
	clearbit(ADMUX,MUX4);

	ADCSRB &= ~((1<< MUX5));	//oberstes bit MUX5 (in anderem Register) ist für single channel ADC0-7 immer 0! //MUX5 =3 (bit 3)

	setbit(ADCSRA,ADEN);	// ADC enable
	setbit(ADCSRA,ADIE);	// ADC interrupt enable
	setbit(ADCSRA,ADSC);	// start first ADC conversion

}

// alle ADC-Daten zurückmelden <ui:*nummer*:*wert*>
// txtbuffer muss UART_MAXSTRLEN+1 groß sein
void adc_msg_all(char *txtbuffer)
{
	uint8_t i;
	char valbuffer[10];
	for( i = 0; i < 8; i++ )
	{
		if ((1<<i) & adc_mask)	// wenn channel verwendet wird
		{
			memset(txtbuffer, 0, UART_MAXSTRLEN+1);	// text leeren
			strlcpy_P(txtbuffer, txtp_cmd_ui, 5);
			itoa(i, valbuffer, 10);
			strlcat(txtbuffer, valbuffer, UART_MAXSTRLEN+1);	// will länge des kompletten "test" buffers+0
			strlcat_P(txtbuffer, txtp_cmdtrenn, UART_MAXSTRLEN+1);
			itoa(adcvalue[i], valbuffer, 10);
			strlcat(txtbuffer, valbuffer, UART_MAXSTRLEN+1);
			strlcat_P(txtbuffer, txtp_cmdend, UART_MAXSTRLEN+1);
			wlan_puts(txtbuffer);
		}

	}

}

// ADC-Daten auswerten (nachdem alle Messungen des aktuellen Channels durchgeführt wurden)
// und auf nächsten Channel umkonfigurieren und Messung starten
void check_adc()
{
	// gemessene Daten auswerten

	adcvalue[adcchannel] = adcvalue_work >> 2;	// die 4 aufaddierten Werte wieder durch 4 dividieren

	// nächsten Channel vorbereiten und Messung starten
	adcvalue_work = 0;
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

	// ADMUX = (ADMUX & ~(0x1F)) | (1<<adcchannel & 0x1F);

	// nächsten Channel einstellen
	uint8_t i;
	for (i = 0; i < 3; i++ )
	{
		if (adcchannel & 1<<i) { setbit(ADMUX,i); }
		else { clearbit(ADMUX,i); }
	}


	cli();
	state &= ~STATE_ADC_CHECK;	// state-flag zurücksetzen
	sei();

	setbit(ADCSRA,ADSC);	// start next conversion
}



// ADC Interrupt routine

ISR(ADC_vect) {

	unsigned int val;

	// var:
	// volatile uint8_t adcchannel = 0;			// aktueller ADC channel (0-7)
	// volatile uint8_t adcreadcount = 0;		// counter für Lesevorgänge pro ADC channel
	// volatile unsigned int adcvalue[adcchannel][adcreadcount-1] // 8 cannels, 4 Werte werden gelesen (der erste wird ignoriert!)

	// auslesen und in richtiger Variable speichern
	// 3 oder 5 werte lesen, den 1. verwerfen
	// danach auf nächsten port umstellen
	// dann in der Hauptschleife den Wert mitteln


	val = ADC;	//Wert muss immer gelesen werden

	PORT_TESTSIGNAL ^= (1<<PD6);	// TODO: nur für test!

	if (adcreadcount > 0) { adcvalue_work += val; }	// erste Messung immer ignorieren, die weitern 4 aufaddieren (zur späteren Mittelung)

	adcreadcount++;

	if (adcreadcount > 4) { state |= STATE_ADC_CHECK; }	// state setzen: Messdaten können ausgewertet werden
	else { setbit(ADCSRA,ADSC); } 	// start next conversion

}	// ISR end


