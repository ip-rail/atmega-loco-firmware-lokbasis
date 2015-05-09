/*
 * commands.c
 *
 *  Created on: 08.12.2010 - 02.05.2015
 *      Author: Michael Brunnbauer
 */

#include <avr/io.h>
#include <string.h>		// f�r "strcmp"
#include <stdlib.h>		// f�r "itoa"
#include <util/delay.h>	// f�r delay_ms()
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>

#include "lokbasis_hwdef.h"		// Hardware-Definitionen f�r die verschiedenen Boards
#include "main.h"
#include "commands.h"
#include "uart.h"
#include "wlan.h"
#include "funktionen.h"	// Funktionen f�r Hauptschleife und commands.c


//-----------------------------------------------------------------------------------------
// befehl_auswerten: wlan_string wird ausgewertet und der Befehl umgesetzt
// es darf nur ein einziger g�ltiger Befehl vorhanden sein -> daher aufruf aus check_wlan_cmd()
// die <> sind bereits entfernt!!!
//-----------------------------------------------------------------------------------------
void befehl_auswerten(void)
{
	char test[UART_MAXSTRLEN+1];
	memset(test, 0, UART_MAXSTRLEN+1);	// text leeren
	alivecount++;	// Meldungen von der Gegenstelle z�hlen (wenn es kein g�ltiger Befehl war, am Ende wieder --;

	uint8_t cmd_found = 1;	// checken, ob cmd behandelt wurde oder nicht

	if(!strcmp(wlan_string, "stop"))
	{
		cli();
		speed_soll = 0;
		sei();
	}

	else if(!strcmp(wlan_string, "off"))
	{
			cli();
			speed_soll = 0;
			sei();
	}

	else if(!strcmp(wlan_string, "stopall"))
		{
				cli();
				speed_soll = 0;
				sei();
		}

	else if(!strncmp(wlan_string, "richtung:", 9))
	{
		strncpy(test, wlan_string+9, 2);
		test[2] = (char) 0;

		if(!strcmp(test, "vw"))		// vorw�rts
		{
			cli();
			richtung_soll = RICHTUNG_VW;
			sei();
		}
		else if(!strcmp(test, "rw"))
		{
			cli();
			richtung_soll = RICHTUNG_RW;
			sei();
		}
	}

	else if(!strncmp(wlan_string, "sd:", 3))	// Command f�r speed setzen
	{
		strncpy(test, wlan_string+3, strlen(wlan_string+3)); 		// die beliebig lange Zahl rauskopiern

		cli();
		speed_soll = atoi(test);
		sei();
	}

	else if(!strcmp(wlan_string, "ping"))	// mit "<pong>" antworten
	{
		uart0_puts("<pong>");
	}

	else
	{
		cmd_found = 0;
		// Bearbeite ung�ltiges Kommando
		// TODO: was machen?
		alivecount--;	// den Z�hler korrigieren (wurde bereits im vorhinein erh�hlt) - es werden nur g�ltige Befehle gez�hlt!!
	}

	/*
	if (cmd_found)
	{
		// Test-Ausgabe auf's LCD
		//lcd_command(LCD_CLEAR);
		//lcd_printlc(1,1,(unsigned char *)"letzter Befehl:");
		//lcd_printlc(2,1,(unsigned char *)wlan_string);
	} */

}
