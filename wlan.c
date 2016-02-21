/*
 * wlan.c
 *
 *  Created on: 29.01.2011 - 02.05.2015
 *      Author: Michael Brunnbauer
 *      Funktionen für die verschiedenen WLAN-Module
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
#include "commands.h"
#include "wlan.h"
#include "funktionen.h"


// Zwischenlösung Auswahl UART für WLAN (0 / 1)
// WLAN_UART_NR definiert in lokbasis_hwdef.h

void wlan_puts_p(const char *progmem_s )
{
	#if defined( WLAN_UART_NR )	// WLAN_UART_NR = 1
		uart1_puts_p(progmem_s);
	#else // WLAN_UART_NR = 0
		uart0_puts_p(progmem_s);
	#endif	// WLAN_UART_NR

}

void wlan_puts(const char *s )
{
	#if defined( WLAN_UART_NR )	// WLAN_UART_NR = 1
		uart1_puts(s);
	#else // WLAN_UART_NR = 0
		uart0_puts(s);
	#endif	// WLAN_UART_NR
}

// check_wlan_cmd(): Funktion für zyklischen Check nach WLAN-Befehlen für die Hauptschleife.
// Übernimmt die vorhandene Daten pro Befehl (also Text zwischen < und >) nach wlan_string
// Es wird nicht auf einen vollständigen Befehl gewartet.
// Ist ein Befehl vollständig, wird die Ausführung des Befehls aufgerufen -> befehl_auswerten()

void check_wlan_cmd()
{
	static unsigned char cmdstate = WLANCMD_NONE;	// Status des aktuellen Befehls
	static unsigned char cindex = 0;		// Position in wlan_string (global)
	unsigned int c;						// zum Verarbeiten des aktuellen Zeichens
	unsigned char exit;					// für while-Schleifen-Ausstieg
	unsigned char uarterror;			// UART Empfangsfehler

	exit = 0;

	while(!exit)
	{
		// uart0_getc() returns in the lower byte the received character and in the higher byte (bitmask) the last receive error
		// UART_NO_DATA is returned when no data is available.

		uarterror = 0;
		//c = uart0_getc();	//

		#if defined( WLAN_UART_NR )	// WLAN_UART_NR = 1
			c = uart1_getc();
		#else // WLAN_UART_NR = 0
			c = uart0_getc();
		#endif	// WLAN_UART_NR




		if ( c & UART_NO_DATA )	//no data available from UART
		{
			exit = 1;
		}
		else	// data available
		{

			// new data available from UART - check for Frame or Overrun error

			if ( c & UART_FRAME_ERROR )
			{
				// Framing Error detected, i.e no stop bit detected
				uarterror = 1;
			}
			if ( c & UART_OVERRUN_ERROR )
			{
				// Overrun, a character already present in the UART UDR register was
				// not read by the interrupt handler before the next character arrived,
				// one or more received characters have been dropped
				uarterror = 2;
			}
			if ( c & UART_BUFFER_OVERFLOW )
			{
				// We are not reading the receive buffer fast enough,  one or more received character have been dropped
				uarterror = 3;
			}

			// empfangenes Zeichen verarbeiten

			if (!uarterror)	// falls kein Fehler aufgetreten ist
			{
				char d = (char)c;

				if (d == 60)	// > Befehl beginnt
				{

					cmdstate = WLANCMD_STARTED;	// ab nun Zeichen nach wlan_string übernehmen
					cindex = 0;
					memset(wlan_string, 0, UART_MAXSTRLEN+1);
				}
				else if (d == 62)	// > abschließendes Zeichen wurde empfangen
				{
					if (cmdstate == WLANCMD_STARTED) { befehl_auswerten(); }
					cmdstate = WLANCMD_NONE;
				}
				else
				{
					if (cmdstate == WLANCMD_STARTED)	// Zeichen in Befehl übernehmen
					{
						if (cindex < UART_MAXSTRLEN)
						{
							wlan_string[cindex] = d;
							cindex++;
						}
						else { cmdstate = WLANCMD_NONE; }	// Befehl ist zu lange - muss ignoriert werden!
					}
				}
			}
			else { cmdstate = WLANCMD_NONE; }	// bei uarterror bisherige Befehlsdaten verwerfen
		}


	} // end while loop


} //end check_wlan_cmd()


