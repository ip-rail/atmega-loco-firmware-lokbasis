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
#include "servo.h"

//-----------------------------------------------------------------------------------------
// befehl_auswerten: wlan_string wird ausgewertet und der Befehl umgesetzt
// es darf nur ein einziger gültiger Befehl vorhanden sein -> daher aufruf aus check_wlan_cmd()
// die <> sind bereits entfernt!!!
//-----------------------------------------------------------------------------------------
void befehl_auswerten(void)
{
	char test[UART_MAXSTRLEN+1];
	uint8_t cmd_found = 1;	// checken, ob cmd behandelt wurde oder nicht

	memset(test, 0, UART_MAXSTRLEN+1);	// text leeren
	alivecount++;	// Meldungen von der Gegenstelle zählen (wenn es kein gültiger Befehl war, am Ende wieder --;
	if (alivecount == 0) { alivecount++; }	// damit nach Überlauf der alivecount nicht 0 wird


	if(!strcmp_P(wlan_string, txtp_cmd_stop))	// "stop"
	{
		speed_soll = 0;
	}

	else if(!strcmp_P(wlan_string, txtp_cmd_off))	// "off"
	{
		speed_soll = 0;
	}

	else if(!strcmp_P(wlan_string, txtp_cmd_stopall))	// "stopall"
	{
		speed_soll = 0;
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
		speed_soll = zahl;
	}

	else if(!strcmp_P(wlan_string, txtp_cmd_ping))	// "ping" - mit "<pong>" antworten
	{
		wlan_puts_p(txtp_pong);
	}

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

	else if(!strncmp_P(wlan_string, txtp_cmd_nameset, 8))	// "nameset:" set loco name in eeprom
	{
		static char datatxt[EEDATA_MAXSTRLEN];

		if (!strncmp_P(wlan_string+8, txtp_cmddata_start, 6))	// "start:"
		{
			memset(datatxt, 0, EEDATA_MAXSTRLEN);	// string leeren
			strncpy(datatxt, wlan_string+14, strlen(wlan_string+14));
		}
		else if(!strncmp_P(wlan_string+8, txtp_cmddata_start, 4))	// "add"
		{
			strlcat(datatxt, wlan_string+12, EEDATA_MAXSTRLEN); // (will Länge des kompletten destination buffers+0!!)
		}
		else if(!strncmp_P(wlan_string+8, txtp_cmddata_start, 4))	// "end"
		{
			strlcat(datatxt, wlan_string+12, EEDATA_MAXSTRLEN);
			//datatxt enthält jetzt den kompletten Namen!
			eeprom_update_lname(datatxt);
			// TODO: wird im Betrieb auch benötigt??
		}
	}

	else if(!strcmp_P(wlan_string, txtp_cmd_hwget))	{ wlan_puts_p(txtp_hwi); }	//  für das UC02 Board mit "<hwi:01>" antworten

	else if(!strcmp_P(wlan_string, txtp_cmd_servoget))		// servoget - Servo-Konfiguration wird angefordert
	{
		uint8_t i;
		char strbuffer[10];
		strlcpy_P(test, txtp_cmd_servoi, 9);	// Rückmeldung per <servoi:mode:count:*port0**pin0*:..>
		itoa(servo_mode, strbuffer, 10);
		strlcat(test, strbuffer, UART_MAXSTRLEN+1);	// will länge des kompletten "test" buffers+0
		strlcat_P(test, txtp_cmdtrenn, UART_MAXSTRLEN+1);
		itoa(servo_count, test+9, 10);

		for( i = 0; i < servo_count; i++ )
		{
			strlcat_P(test, txtp_cmdtrenn, UART_MAXSTRLEN+1);
			strbuffer[0] = servoPort[i];
			strbuffer[1] = servoPin[i] + 48;	// einfache ASCII-Konvertierung "0" ist ASCII-Code 48
			strbuffer[1] = 0;					//String Terminierung
			strlcat(test, strbuffer, UART_MAXSTRLEN+1);
		}
		strlcat_P(test, txtp_cmdend, UART_MAXSTRLEN+1);
		wlan_puts(test);
	}

	else if(!strcmp_P(wlan_string, txtp_cmd_servoset))		// servoset  zB: "servoset:0:6:B0:B1:B2:B3:B7:D4"
	{
		uint8_t mode, count, i;
		mode = wlan_string[9] - 48;	// mögliche Werte: 0 oder 1
		count = wlan_string[11];	// mögliche Werte: 0 bis SERVOCOUNTMAX

		if ((mode > 1) || (count > SERVOCOUNTMAX)) { return; }// Daten stimmen nicht

		servo_mode = mode;
		servo_count = count;
		char* checkstring;
		checkstring = wlan_string + 12;

		for( i = 0; i < count; i++ )
		{
			checkstring++;	// ':' ignorieren
			servoPort[i] = *checkstring++;
			servoPin[i] = (*checkstring++) - 48;
		}

		//Saving new settings to EEPROM
		eeprom_update_ServoMode(mode);
		eeprom_update_ServoCount(count);
		eeprom_update_ServoGPIO(servoPort, servoPin);

		//TODO: neu initialisieren? (zumindest teilweise weil sich womöglich GPIOs geändert haben? -> checken)
	}

	
	// <gpioc:pin:wert> pin: 0-7, wert: 0 oder 1
	else if(!strncmp_P(wlan_string, txtp_cmd_gpioc, 6))	// "gpioc:" - <gpioc:pin:wert> pin: 0-7, wert: 0 oder 1
	{
		uint8_t pinnr,pinval;
		strncpy(test, wlan_string+6, 1);	//nur 1 Zeichen auswerten (pin)
		pinnr = (uint8_t)atoi(test);

		memset(test, 0, UART_MAXSTRLEN+1);	// text wieder leeren
		strncpy(test, wlan_string+8, 1);	//nur 1 Zeichen auswerten (wert)
		pinval = (uint8_t)atoi(test);

		if (pinval) { PORTC |= (1<<pinnr); }	// Pin auf 1 setzen
		else { PORTC &= ~(1<<pinnr); }			// Pin auf 0 setzen

	}

	else if(!strncmp_P(wlan_string, txtp_cmd_fpwmset, 8))	// "fpwmset:"  PWM-Frequenz setzen
	{
		uint8_t pwmf_new = 0;
		strncpy(test, wlan_string+8, 1);		// die einstellige Zahl rauskopiern
		test[1] = (char) 0;	// string mit 0-Byte abschließen
		pwmf_new = atoi(test);

		if ((pwmf_new < 1) || (pwmf_new > 9)) { pwmf_new = MOTOR_PWMF_STD; } // muss im Berech on 1 bis 9

		if (pwmf_new != motor_pwmf)
		{
			motor_pwmf = pwmf_new;
			eeprom_update_MotorPWMf(motor_pwmf);
			speed_soll = 0;
			if (speed != 0) { warte_ms(1000); }	// zur Sicherheit warten, bis Lok steht -> 1s Zeit geben
			init_pwm(motor_pwmf);		// neuen PWM-Modus setzen
		}
	}

	else if(!strcmp_P(wlan_string, txtp_cmd_fpwmget))	// <fpwmget> PWM-Frequenz abfragen -> Rückmeldung <fpwmi:n>
	{
		// txtp_cmd_fpwmi[] PROGMEM = "<fpwmi:";
		strlcpy_P(test, txtp_cmd_fpwmi, 8);	// Rückmeldung der Geschwindigkeit	//bei der Länge immer 0-Zeichen mitzählen!
		itoa(motor_pwmf, test+7, 10);
		strlcat_P(test, txtp_cmdend, UART_MAXSTRLEN+1);	// will länge des kompletten "test" buffers+0
		wlan_puts(test);
	}



	// TODO: ADCGPIO (adc_used) set,get,rückmeldung
	// TODO: konfigurierbare GPIOs set,get,rückmeldung
	// TODO: switch Befehle zum Schalten freier GPIOs
	// TODO: alive-fuktion ja/nein, alive-interval


	else
	{
		cmd_found = 0;
		// Bearbeite ungültiges Kommando
		// TODO: was machen? Rückmeldung unbekannter Befehl
		alivecount--;	// den Zähler korrigieren (wurde bereits im vorhinein erhöhlt) - es werden nur gültige Befehle gezählt!!
		// weil ungültige Befehle helfen nicht, wenn die Lok amok-läuft
	}

	/*
	if (cmd_found)
	{
		// Test-Ausgabe auf's LCD
		//lcd_command(LCD_CLEAR);
		//lcd_printlc(1,1,(unsigned char *)"letzter Befehl:");
		//lcd_printlc(2,1,(unsigned char *)wlan_string);
	} */

	// TODO: ev. Fehler rückmelden, wenn Befehl nicht bekannt?

}
