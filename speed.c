/*
 * speed.c
 *
 *  Created on: 01.08.2010 - 02.05.2015
 *      Author: Michael Brunnbauer
 */

#include <avr/io.h>
#include <string.h>		// f�r "strcmp"
#include <stdlib.h>		// f�r "itoa"
#include <util/delay.h>	// f�r delay_ms()
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "lokbasis_hwdef.h"
#include "uart.h"
#include "speed.h"
#include "main.h"


//----------------------------------------------------------------------------------------------
// set_speed
// setzt als einzige Funktion den PWM Wert f�r die Geschwindigkeit!!!
// und gl�ttet die Geschwindigkeitsspr�ge vom Steuerungsger�t bei Start / Stop / starken Beschleunigungen
//----------------------------------------------------------------------------------------------

void set_speed(void)
{
	int speeddelta = 0;

	//---------- Anfangszustand: Speed�nderung von 0 -> Richtung am Motor setzen --------------------
	// Richtungswechsel nur bei Speed == 0, daf�r immer, wenn Speed-�nderung vorliegt
	if ((speed == 0) & (speed_soll != 0))
	{
		if(richtung_soll == RICHTUNG_VW)		// vorw�rts
		{

#if defined( HW_TESTBOARD2 ) || defined( PHB01_MOTOR1 )
			setbit(PORT_MOTOR, MOTOR1_DIR);	// Motor1 vorw�rts
#endif

#if defined( PHB01_MOTOR2 )
			setbit(PORT_MOTOR, MOTOR2_DIR);	// Motor2 (eigene H-Br�cke) vorw�rts
#endif


			richtung = RICHTUNG_VW;
		}
		else if(richtung_soll == RICHTUNG_RW)	// r�ckw�rts
		{

#if defined( HW_TESTBOARD2 ) || defined( PHB01_MOTOR1 )
			clear(PORT_MOTOR, MOTOR1_DIR);	// Motor1 r�ckw�rts
#endif

#if defined( PHB01_MOTOR2 )
			clear(PORT_MOTOR, MOTOR2_DIR);	// Motor2 (eigene H-Br�cke) vorw�rts
#endif

			richtung = RICHTUNG_RW;
		}
	}


	//----- Geschwindigkeits�nderung ------------------------------------------------------------------------------
	// TODO: Geschwindigkeits�nderung sollte verfeinert werden!

	if (richtung != richtung_soll)	// Richtungswechsel: speed auf 0 reduzieren
	{
		if(speed != 0)	// Richtungswechsel -> vorher stop!
		{
			speed = speed - speed / 5;
			if (speed > 50) { speed = speed - speed / 5;	}
			else { speed = 0; }
		}
	}

	if (speed != speed_soll)
	{
		if (richtung == richtung_soll)	// Richtung passt -> beschleunigen
		{
			speeddelta = speed_soll - speed;
			if (abs(speeddelta) > 25) { speed = speed + speeddelta / 5; }
			else { speed = speed_soll; }
		}
	}


	//---- PWM-Wert setzen -------------------------------------------------------------------------
	// wird nur hier ausgef�hrt, sonst nirgends mehr!!!!!


#if defined( MC_INV_PWM_4_REV )	// PWM-Wert mu� f�r R�ckw�rts invertiert werden

	if (richtung == RICHTUNG_VW) // vorw�rts
	{
		OCR1A = (speed>>speedstep_korrektur);
		OCR1B = (speed>>speedstep_korrektur);
		pwm =   (speed>>speedstep_korrektur);
	}
	else if (richtung == RICHTUNG_RW)	// r�ckw�rts
	{
		switch (speedstep_korrektur)
		{
		case 0:	// TOP = 1000 (nicht 1023 wg. dreistelligem speed-wert im command!)	// ab v0.15 auch bis 1023 m�glich!
			OCR1A = 1023 - speed;
			OCR1B = 1023 - speed;
			pwm =   1023 - speed;
			break;

		case 1:	// TOP = 511
			OCR1A = 511 - (speed>>speedstep_korrektur);
			OCR1B = 511 - (speed>>speedstep_korrektur);
			pwm   = 511 - (speed>>speedstep_korrektur);
			break;

		case 2:	// TOP = 255
			OCR1A = 255 - (speed>>speedstep_korrektur);
			OCR1B = 255 - (speed>>speedstep_korrektur);
			pwm   = 255 - (speed>>speedstep_korrektur);
			break;
		}
	}
#else	// gleicher PWM-Wert f�r alle Richtungen
	MOTOR1_OCR = (speed>>speedstep_korrektur);

	#if defined( PHB01_MOTOR2 )
	MOTOR2_OCR = (speed>>speedstep_korrektur);
	#endif	// PHB01_MOTOR2

#endif	// MC_INV_PWM_4_REV

}

