/*
 * commands.c
 *
 *  Created on: 08.12.2010 - 02.05.2015
 *      Author: Michael Brunnbauer
 */

#include <avr/io.h>
#include <string.h>		// für "strcmp"
#include <stdlib.h>		// für "itoa"
#include <util/delay.h>	// für delay_ms()
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>	//für Watchdog Reset

#include "lokbasis_hwdef.h"		// Hardware-Definitionen für die verschiedenen Boards
#include "main.h"
#include "commands.h"
#include "uart.h"
#include "wlan.h"
#include "funktionen.h"	// Funktionen für Hauptschleife und commands.c
#include "ledc.h"
#include "eedata.h"

//-----------------------------------------------------------------------------------------
// befehl_auswerten: wlan_string wird ausgewertet und der Befehl umgesetzt
// es darf nur ein einziger gültiger Befehl vorhanden sein -> daher aufruf aus check_wlan_cmd()
// die <> sind bereits entfernt!!!
//-----------------------------------------------------------------------------------------
void befehl_auswerten(void)
{
	char test[UART_MAXSTRLEN+1];
	memset(test, 0, UART_MAXSTRLEN+1);	// text leeren
	alivecount++;	// Meldungen von der Gegenstelle zählen (wenn es kein gültiger Befehl war, am Ende wieder --;

	uint8_t cmd_found = 1;	// checken, ob cmd behandelt wurde oder nicht

	if(!strcmp_P(wlan_string, txtp_cmd_stop))	// "stop"
	{
		cli();
		speed_soll = 0;
		sei();
	}

	else if(!strcmp_P(wlan_string, txtp_cmd_off))	// "off"
	{
		cli();
		speed_soll = 0;
		sei();
	}

	else if(!strcmp_P(wlan_string, txtp_cmd_stopall))	// "stopall"
	{
		cli();
		speed_soll = 0;
		sei();
	}

	else if(!strncmp_P(wlan_string, txtp_cmd_richtung, 9))	//  "richtung:"
	{
		strncpy(test, wlan_string+9, 2);
		test[2] = (char) 0;

		if(!strcmp_P(test, txtp_cmddata_vw))		// "vw" - vorwärts
		{
			cli();
			richtung_soll = RICHTUNG_VW;
			sei();
		}
		else if(!strcmp_P(test, txtp_cmddata_rw))	// "rw" - rückwärts
		{
			cli();
			richtung_soll = RICHTUNG_RW;
			sei();
		}
	}

	else if(!strncmp_P(wlan_string, txtp_cmd_sd, 3))	// "sd:" Command für speed setzen
	{
		int zahl = 0;
		strncpy(test, wlan_string+3, strlen(wlan_string+3)); 		// die beliebig lange Zahl rauskopiern

		zahl = atoi(test);
		if ((zahl < 0) || (zahl > 1023)) { zahl = 0; }		// wenn etwas nicht passt

		cli();
		speed_soll = zahl;
		sei();
	}

	else if(!strcmp_P(wlan_string, txtp_cmd_ping))	{ uart0_puts_p(txtp_pong); }	// "ping" - mit "<pong>" antworten

	else if(!strncmp_P(wlan_string, txtp_cmd_l1, 3))	// "l1:" - <l1:*nummer*> Licht einschalten: Licht *Nummer* 1-16
	{
		uint8_t lednr;
		strncpy(test, wlan_string+3, strlen(wlan_string+3)); 
		lednr = (uint8_t)atoi(test);
		ledc_led_setpwm(LEDC1, lednr, 255);
	}

	else if(!strncmp_P(wlan_string, txtp_cmd_l0, 3))	// "l0:" - <l0:*name*> Licht ausschalten: Licht *Nummer* 1-16
	{
		uint8_t lednr;
		strncpy(test, wlan_string+3, strlen(wlan_string+3)); 
		lednr = (uint8_t)atoi(test);
		ledc_led_setpwm(LEDC1, lednr, 0);
	}

	else if(!strcmp_P(wlan_string, txtp_cmd_reset))	// "reset" for starting bootloader
	{
		cli();
		wdt_enable(WDTO_15MS);	// activate watchdog timer and
		while (1);				// wait for watchdog reset
	}

	else if(!strncmp_P(wlan_string, txtp_cmd_onameset, 9))	// "onameset:" set owner name in eeprom
	{
		static char datatxt[EEDATA_MAXSTRLEN];

		if (!strncmp_P(wlan_string+9, txtp_cmddata_start, 6))	// "start:"
		{
			memset(datatxt, 0, EEDATA_MAXSTRLEN);	// string leeren
			strncpy(datatxt, wlan_string+15, strlen(wlan_string+15));

		}
		else if(!strncmp_P(wlan_string+9, txtp_cmddata_start, 4))	// "add"
		{
			strlcat(datatxt, wlan_string+13, EEDATA_MAXSTRLEN); // (will Länge des kompletten destination buffers+0!!)
		}
		else if(!strncmp_P(wlan_string+9, txtp_cmddata_start, 4))	// "end"
		{
			strlcat(datatxt, wlan_string+13, EEDATA_MAXSTRLEN);
			//datatxt enthält jetzt den kompletten Namen!
			eeprom_update_oname(datatxt);
			// TODO: wird im Betrieb auch benötigt??
		}

	}
	
	
	// TODO: switch Befehle zum Schalten freier GPIOs






	else
	{
		cmd_found = 0;
		// Bearbeite ungültiges Kommando
		// TODO: was machen? Rückmeldung unbekannter Befehl
		alivecount--;	// den Zähler korrigieren (wurde bereits im vorhinein erhöhlt) - es werden nur gültige Befehle gezählt!!
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
