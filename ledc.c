/*
 * ledc.c
 *
 *  Created on: 16.05.2015
 *      Author: Michael Brunnbauer
 *
 *      I2C LED-Controller PCA9622,TLC59116
 */

#include <avr/io.h>
//#include <string.h>		// für "strcmp"
//#include <stdlib.h>		// für "itoa"
#include <util/delay.h>	// für delay_ms()
//#include <avr/interrupt.h>
//#include <avr/pgmspace.h>

#include "lokbasis_hwdef.h"
#include "main.h"
#include "i2cmaster.h"			// I2C Funktionen
#include "funktionen.h"


// I2C, LED-Controller-Funktionen PCA9622,TLC59116

//LED-Controller Software Reset
void ledcontrol_reset()
{
	i2c_start(LEDC_I2C_RESET);		//LED-Controller auf Reset-Adresse ansprechen
	i2c_write(0xA5);				// jetzt 2 Datenbytes A5, 5A, daran erkennt der LED-Chip, dass er gemeint ist und resettet sich nach dem i2c_stop()
	i2c_write(0x5A);
	i2c_stop();						//TWI anhalten
}

//LED-Controller Initialisieren
void ledcontrol_init(uint8_t slaveaddr)
{
	uint8_t i;
	uint8_t count = 0;
	unsigned char error;

	do	// erfolgreichen Start versuchen
	{
		if (count > 0) { warte_ms(50); }		// kleine Pause, wenn Salve nicht bereit
		count++;
		error = i2c_start(LEDC1+I2C_WRITE);		// return 0 = device accessible, 1= failed to access device
	} while (error && (count < 3));				// nochmal versuchen, wenn Slave nicht bereit, aber nur 3 Versuche


	if(!error)								//Start: LEDcontroller auf globaler Adresse im Write-Modus ansprechen
	{
		i2c_write(0x80);					//Controllregister ansprechen (0x80 = autoincrement for all registers, bei 00 beginnend)
		i2c_write(LEDC_MODE1);				//Mode1 auf Normal setzen (subaddresses deaktivieren, all-call-address aktivieren)
		i2c_write(LEDC_MODE2_BLINK);		//Mode2 auf Default setzen (0x00: group controll= dimming, 0x10: group controll= blinking)
		for(i=0;i<16;i++)
		{
			i2c_write(0);					//alle PWM Kanäle auf 0 (Helligkeit)
		}
		i2c_write(0xff);						//PWM Helligkeit fuer Gruppen auf maximum
		i2c_write(11);						//Gruppen-Blinkfrequenzregister auf 11
		for(i=0;i<4;i++)
		{
			i2c_write(0xff);					//Std: 0:Outputs einschalten
		}
		i2c_stop();							//TWI Stoppen	// IREF Regidster muss nicht gesetzt werden (Defaults: alles auf 1 passt)
	}
}

//LED Helligkeit (1-16) setzen 
void ledc_led_setpwm(int slaveaddr, uint8_t lednumber, uint8_t value)
{
	uint8_t count = 0;
	unsigned char error;
	
		do	// erfolgreichen Start versuchen
	{
		if (count > 0) { warte_ms(50); }		// kleine Pause, wenn Salve nicht bereit
		count++;
		error = i2c_start(LEDC1+I2C_WRITE);		// return 0 = device accessible, 1= failed to access device
	} while (error && (count < 3));				// nochmal versuchen, wenn Slave nicht bereit, aber nur 3 Versuche

	if(!error)	
	{
		i2c_write(PCA9622_ADDR_PWM0+lednumber-1);			//Controllregister ansprechen
		i2c_write(value);								// Helligkeitswert schreiben
		
		i2c_stop();	
	}
}





