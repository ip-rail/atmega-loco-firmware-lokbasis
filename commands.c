/*
 * commands.c
 *
 *  Created on: 08.12.2010 - 02.05.2015
 *      Author: Michael Brunnbauer
 */


#include <avr/interrupt.h>
#include <avr/iomxx0_1.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <stdint.h>
#include <stdio.h>		// für "sprintf"
#include <stdlib.h>		// für "itoa"
#include <string.h>			// für "strcmp"

#include "commands.h"
#include "eedata.h"
#include "funktionen.h"	// Funktionen für Hauptschleife und commands.c#include "ledc.h"#include "ledc.h"
#include "lokbasis_hwdef.h"		// Hardware-Definitionen für die verschiedenen Boards#include "main.h"#include "main.h"
#include "servo.h"
#include "wlan.h"

//-----------------------------------------------------------------------------------------
// befehl_auswerten: wlan_string wird ausgewertet und der Befehl umgesetzt
// es darf nur ein einziger gültiger Befehl vorhanden sein -> daher aufruf aus check_wlan_cmd()
// die <> sind bereits entfernt!!!
// TODO: strlcpy,strlcat: https://www.sudo.ws/todd/papers/strlcpy.html statt strncpy..
// TODO: checken, wo man (für Rückmeldungen) auf sprintf_P() umbauen kann
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
		eeprom_update_oname(wlan_string+9);
	}

	else if(!strcmp_P(wlan_string, txtp_cmd_onameget))	// "onameget"
	{
		strlcpy_P(test, txtp_cmd_onamei, sizeof(test));
		eeprom_get_oname(test+8);
		strlcat_P(test, txtp_cmdend, sizeof(test));	// will länge des kompletten "test" buffers+0
		wlan_puts(test);
	}

	else if(!strncmp_P(wlan_string, txtp_cmd_nameset, 8))	// "nameset:" set loco name in eeprom
	{
		eeprom_update_lname(wlan_string+8);	// TODO: wird lokname im Betrieb auch benötigt??
	}

	else if(!strcmp_P(wlan_string, txtp_cmd_nameget))	// "nameget"
	{
		strlcpy_P(test, txtp_iam, sizeof(test));
		eeprom_get_lname(test+7);
		strlcat_P(test, txtp_cmdend, sizeof(test));	// will länge des kompletten "test" buffers+0
		wlan_puts(test);
	}

	else if(!strcmp_P(wlan_string, txtp_cmd_hwget))
	{
		//wlan_puts_p(txtp_cmd_hwi); //  für das UC02 Board mit "<hwi:01>" antworten
		//wlan_puts_p(txtp_cmd_ntypi);	// Netzwerk-Typ (dummes WLAN-Modul)
		sendHWinfo(test);		// wie bisher + Status für alle GPIO-Ports
	}

	else if(!strcmp_P(wlan_string, txtp_cmd_servoget))		// servoget - Servo-Konfiguration wird angefordert
	{
		uint8_t i;
		char strbuffer[10];
		strlcpy_P(test, txtp_cmd_servoi, sizeof(test));	// Rückmeldung per <servoi:mode:count:*port0**pin0*:..>
		itoa(servo_mode, strbuffer, 10);
		strlcat(test, strbuffer, sizeof(test));	// will länge des kompletten "test" buffers+0
		strlcat_P(test, txtp_cmdtrenn, sizeof(test));
		itoa(servo_count, test+9, 10);

		for( i = 0; i < servo_count; i++ )
		{
			strlcat_P(test, txtp_cmdtrenn, sizeof(test));
			strbuffer[0] = servoPort[i];
			strbuffer[1] = servoPin[i] + 48;	// einfache ASCII-Konvertierung "0" ist ASCII-Code 48
			strbuffer[2] = 0;					//String Terminierung
			strlcat(test, strbuffer, sizeof(test));
		}
		strlcat_P(test, txtp_cmdend, sizeof(test));
		wlan_puts(test);
	}

	else if(!strncmp_P(wlan_string, txtp_cmd_servoset, 9))	// "servoset:"  zB: "servoset:0:6:B0:B1:B2:B3:B7:D4"
	{
		uint8_t mode, count, i;
		mode = wlan_string[9] - 48;	// mögliche Werte: 0 oder 1
		count = wlan_string[11];	// mögliche Werte: 0 bis SERVOCOUNTMAX

		if ((mode > 1) || (count > SERVOCOUNTMAX)) { return; }// Daten stimmen nicht

		if (mode != 0) { mode = 0;}	//TODO: derzeit ist nur servo_mode == 0 erlaub!

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

	else if(!strncmp_P(wlan_string, txtp_cmd_servomove, 10))	// "servomove:" Servo position setzen <servomove:servo:wert>
	{
		uint8_t servonr;
		unsigned int servo_pos;

		wlan_string[11] = 0;	// Zahl als "String" isolieren
		servonr = atoi(wlan_string+10);	// TODO: Achtung: hier ist Servo-Nummer immer nur 1 Zeichen -> falls sich das mal ändert, muss die auswertung felxibel gemacht werden!!
		if ((servonr < 1) || (servonr > SERVOCOUNTMAX))
		{
			wlan_puts("<log:Ungültige Servonr!");
			servonr = 1;
		} // TODO: Fehler rückmelden

		servo_pos = atoi(wlan_string+12);
		if (servo_pos > 16000) { servo_pos = 16000; }	// gültige Werte: 0 bis 16000

		cli();
		servoValue[servonr-1] = servo_pos;	// index = 0 bis SERVOCOUNTMAX-1
		sei();

		sprintf_P(test, txtp_i_servo, servoValue[servonr-1], servonr-1);	//TODO: test
		wlan_puts(test);
	}


	
	// <gpioc:pin:wert> pin: 0-7, wert: 0 oder 1	TODO: veraltet, abgelöst durch: <gpio:port:pin:wert> -> entfernen
	else if(!strncmp_P(wlan_string, txtp_cmd_gpioc, 6))	// "gpioc:" - <gpioc:pin:wert> pin: 0-7, wert: 0 oder 1
	{
		uint8_t pinnr,pinval;
		strncpy(test, wlan_string+6, 1);	//nur 1 Zeichen auswerten (pin)
		pinnr = (uint8_t)atoi(test);

		memset(test, 0, UART_MAXSTRLEN+1);	// text wieder leeren
		strncpy(test, wlan_string+8, 1);	//nur 1 Zeichen auswerten (wert) -> ok, weil vorher test gelöscht wurde!!
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
			// speed_soll = 0;
			// if (speed != 0) { warte_ms(1000); }	// zur Sicherheit warten, bis Lok steht -> 1s Zeit geben
			// TODO: hier warten usw. geht nicht - checken, wie sich das in Fahrt verhält
			init_pwm(motor_pwmf);		// neuen PWM-Modus setzen
		}
	}

	else if(!strcmp_P(wlan_string, txtp_cmd_fpwmget))	// <fpwmget> PWM-Frequenz abfragen -> Rückmeldung <fpwmi:n>
	{
		sprintf_P(test, txtp_cmd_fpwmi, motor_pwmf);
		wlan_puts(test);
	}

	else if(!strcmp_P(wlan_string, txtp_cmd_alive))	// <alive>
	{
		// grundsätzlich reicht es, wenn der Befehl als gültig gezählt wird (sonst: siehe else-Zweig: Zähler wird wieder vermindert)
		//TODO: Auswertungen für Stromsparmodus
	}

	else if(!strcmp_P(wlan_string, txtp_cmd_aliveget))	// "aliveget" Wert für das alive-Timeout -> Rückmeldung <alivei:*sekunden*>
	{
		sprintf_P(test, txtp_cmd_alivei, eeprom_getAliveCheckSecs());
		wlan_puts(test);
	}

	else if(!strncmp_P(wlan_string, txtp_cmd_aliveset, 9))	// "aliveset:*sekunden*"  Wert für das alive-Timeout setzen
	{
		int zahl = 0;
		strncpy(test, wlan_string+9, strlen(wlan_string+9)); 	// die beliebig lange Zahl rauskopiern (Achtung: wenn der buffer nicht leer ist, muss länge+1 angegeben werden!)
		zahl = atoi(test);
		if (zahl < 0) { zahl = 3; }

		if (zahl != maxalivesecs)	// bei Änderung umstellen und speichern
		{
			maxalivesecs = zahl;
			eeprom_update_AliveCheckSecs(maxalivesecs);
		}
	}


	else if(!strcmp_P(wlan_string, txtp_cmd_mcfgget))	// <mcfgget> -> motor_cfg config-byte übermitteln
	{
		strlcpy_P(test, txtp_cmd_mcfgi, sizeof(test));				// "<mcfgi:"
		itoa(motor_cfg, test+7, 10);
		strlcat_P(test, txtp_cmdend, sizeof(test));	// will länge des kompletten "test" buffers+0
		wlan_puts(test);
	}

	else if(!strncmp_P(wlan_string, txtp_cmd_mcfgset, 8))	// "mcfgset:" Setzen einer H-Brücken Konfiguration
	{
		int zahl = 0;
		uint8_t motor_cfg_neu;

		strncpy(test, wlan_string+8, strlen(wlan_string+8)); 		// die beliebig lange Zahl rauskopiern

		zahl = atoi(test);
		motor_cfg_neu = (uint8_t)zahl;

		if ((motor_cfg_neu < 8) || (motor_cfg_neu > 255))
		{
			motor_cfg_neu = MOTOR_CONFIG_1HB; // wenn etwas nicht passt -> Standardwert
			wlan_puts("<log:motor_cfg passt nicht!>");	// TODO: check Befehl für: Fehler rückmelden, wenn die Zahl für motor_cfg nicht passt
		}

		if (motor_cfg_neu != motor_cfg)	// bei Änderung umstellen und speichern
		{
			motor_cfg = motor_cfg_neu;
			eeprom_update_MotorConfig(motor_cfg);
			init_motorctrl(); 	// TODO: checken, wie sich das in Fahrt verhält
		}
	}

	else if(!strncmp_P(wlan_string, txtp_cmd_gpioget, 8))	// txtp_cmd_gpioget "gpioget:" char port
	{
		char port = wlan_string[8];

		if ((port == 'B') || (port == 'C')|| (port == 'D') || (port == 'E') || (port == 'G'))	//nur für gültige Ports -> PortC darf abgefragt werden, ist immer komplett auf Output
		{
			uint8_t gpiopins = getGPIOs(port);
			uint8_t usable_pins = getUsableGPIOs(port);	// für ungültigen Port oder wenn nichts verwendet werden darf wird usable_pins = 0 zurückgegeben
			// TODO: Achtung: für extern müssen von UsableGPIOs auch ServiPins weggefiltert werden, solange das nicht extern berücksichtigt wird!
			usable_pins = filterGPIOMask(port, usable_pins);	// TODO: solange machen, bis das extern gemanagt wird!! auch bei sendHWinfo()
			uint8_t values = getGPIOValues(port);
			sprintf_P(test, txtp_cmd_gpioi, port, usable_pins, gpiopins, values);	//"<gpioi:%i:%i:%i:%i>" <gpioi:port:mögliche:verwendete:werte>    :char:byte-mask:byte-mask:byte-mask
			wlan_puts(test);
		}
	}

	else if(!strncmp_P(wlan_string, txtp_cmd_gpioset, 8))	// "gpioset:" <gpioset:port:pin> GPIO-Pin definieren und aktivieren
	{
		uint8_t pinnr = 0;
		uint8_t error = 0;

		char port = wlan_string[8];
		if (!((port == 'B') || (port == 'D') || (port == 'E') || (port == 'G'))) //nur für gültige Ports. Für Port C ist schon alles auf Ausgang gesetzt
		{
			error = 1;
			wlan_puts_p(txtp_err_gpioset_port);
		}

		strncpy(test, wlan_string+10, 1);	//nur 1 Zeichen auswerten (pin)
		pinnr = (uint8_t)atoi(test);

		if (pinnr > 7)
		{
			error = 1;
			wlan_puts_p(txtp_err_gpioset_pin);
		}

		if (!error)
		{
			uint8_t mask = getGPIOs(port) | filterGPIOMask(port, (1<<pinnr));
			eeprom_update_GPIO(port, mask);

			cli();
			sprintf_P(test, txtp_gpio_save, port, mask);	// TODO Test: <log:save P%c msk %i>
			sei();
			wlan_puts(test);

			switch (port)	//Pin auf Output setzen
			{
			case 'B':
				DDRB |= mask;
				break;

			case 'D':
				DDRD |= mask;
				break;

			case 'E':
				DDRE |= mask;
				break;

			case 'G':
				DDRG |= mask;
				break;
			}
		}
	}

	else if(!strncmp_P(wlan_string, txtp_cmd_gpioclear, 10))	// "gpioclear:" <gpioclear:port:pin> Gegenstück zu <gpioset:port:pin> -> verwendeten GPIO-Pin wieder freigeben!
	{
		uint8_t pinnr = 0;
		uint8_t error = 0;

		char port = wlan_string[10];
		if (!((port == 'B') || (port == 'D') || (port == 'E') || (port == 'G'))) 	//nur für gültige Ports
		{
			error = 1;
			wlan_puts_p(txtp_err_gpioset_port);
		}

		strncpy(test, wlan_string+12, 1);	//nur 1 Zeichen auswerten (pin)
		pinnr = (uint8_t)atoi(test);

		if (pinnr > 7)
		{
			error = 1;
			wlan_puts_p(txtp_err_gpioset_pin);
		}

		if (!error)
		{
			uint8_t mask = getGPIOs(port) | filterGPIOMask(port, (1<<pinnr));
			uint8_t clear_mask = filterGPIOMask(port, (1<<pinnr));	// ungültige Pins werden ausgefiltert, damit der User keine Systemfunktionen deaktiviert!

			mask &= ~clear_mask;

			eeprom_update_GPIO(port, mask);	// neue Konfiguration speichern

			switch (port)	//Pin auf Input setzen (Standardzustand, sicherer)
			{
			case 'B':
				DDRB &= ~clear_mask;
				break;

			case 'D':
				DDRD |= ~clear_mask;
				break;

			case 'E':
				DDRE |= ~clear_mask;
				break;

			case 'G':
				DDRG |= ~clear_mask;
				break;
			}
		}
	}


	else if(!strncmp_P(wlan_string, txtp_cmd_gpio, 5))	// "gpio:" - <gpio:port:pin:wert> port B/C/D/E/G (Großbuchstabe!), pin: 0-7, wert: 0 oder 1
	{
		uint8_t error = 0;
		char port = wlan_string[5];
		uint8_t pinnr,pinval;
		strncpy(test, wlan_string+7, 1);	//nur 1 Zeichen auswerten (pin)
		pinnr = (uint8_t)atoi(test);

		memset(test, 0, UART_MAXSTRLEN+1);	// text wieder leeren
		strncpy(test, wlan_string+9, 1);	//nur 1 Zeichen auswerten (wert) -> ok, weil vorher test gelöscht wurde!!
		pinval = (uint8_t)atoi(test);

		if (pinnr > 7)
		{
			error = 1;
			wlan_puts_p(txtp_err_gpio_pin);
		}

		if (!((port == 'B') || (port == 'C')|| (port == 'D') || (port == 'E') || (port == 'G')))	//nur für gültige Ports -> PortC darf abgefragt werden, ist immer komplett auf Output
		{
			error = 1;
			wlan_puts_p(txtp_err_gpio_port);
		}

		uint8_t gpiomask = filterGPIOMask(port, (1<<pinnr));	// pin erlaubt?

		if ((!error) && (gpiomask))
		{
			setGPIOPin(port, pinnr, pinval);	// Ausgabe auf Pin

			// port-status rückmelden
			uint8_t gpiopins = getGPIOs(port);
			uint8_t usable_pins = getUsableGPIOs(port);	// für ungültigen Port oder wenn nichts verwendet werden darf wird usable_pins = 0 zurückgegeben
			uint8_t values = getGPIOValues(port);
			sprintf_P(test, txtp_cmd_gpioi, port, usable_pins, gpiopins, values);	//"<gpioi:%i:%i:%i:%i>" <gpioi:port:mögliche:verwendete:werte>    :char:byte-mask:byte-mask:byte-mask
			wlan_puts(test);
		}
	}


	// TODO: ADCGPIO (adc_used) set,get,rückmeldung
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
