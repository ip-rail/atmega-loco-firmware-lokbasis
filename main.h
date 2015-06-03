/*
 * main.h
 *
 *  Created on: 16.11.2010 - 08.05.2015
 *      Author: Michael Brunnbauer
 */

#ifndef MAIN_H_
#define MAIN_H_


//#define LOKBASIS_NO_TEST	// f�r Testmodus remmen - diverse Sicherheitsabfragen werden f�r's Testen deaktiviert


// Macros f�r Bit-Operationen
#define setbit(P,BIT) 	((P) |= (1<<(BIT)))
#define clearbit(P,BIT)	((P) &= ~(1<<(BIT)))

#define UART_MAXSTRLEN 32		// maximal erlaubte L�nge eines Befehls "<*>" incl. <>

#define ALIVE_INTERVAL 5		//Pr�fintervall = 5 Sekunden! -> Lok stoppt nach 5sek herrenloser Fahrt

// Statusdefinitionen f�r Interrupts und Main-Schleife
#define STATE_1X_PRO_SEK 1		// Timer meldet sich 1x pro Sekunde
#define STATE_5X_PRO_SEK 2		// Timer meldet sich 5x pro Sekunde
#define STATE_3 4				//
#define STATE_4 8				//
#define STATE_5 16				//
#define STATE_6 32				//
#define STATE_7 64				//
#define STATE_8 128				//


// definitionen f�r Steuerung
#define RICHTUNG_VW	1		// Richtung: vorw�rts = 1
#define RICHTUNG_RW	0		// Richtung: r�ckw�rts = 0

// extern verwendete Variablen
extern volatile unsigned char state;		// Status-Variable f�r Interrupts und Main-Schleife
extern unsigned int speed;					// Geschwindigkeitswert der an die H-Br�cke ausgegeben wird (momentaner Wert)
extern unsigned int speed_soll;				// Geschwindigkeitsvorgabe vom Steuerungsger�t (Wert, der erreicht werden soll)
extern unsigned char richtung;
extern unsigned char richtung_soll;
extern unsigned char speedstep_korrektur;
extern volatile unsigned char motor_reg;	// Variable f�r Motor-Regelung ein/aus (auch f�r isr verwendet)
extern uint8_t motorerror;					// Errorcode von Motorcontroller: 0 = kein Error
extern unsigned char alivecount;			// z�hlt die empfangen Meldungen der Gegenstelle (wird zyklisch ausgewertet)
extern volatile char alone;
extern char wlan_string[UART_MAXSTRLEN+1];  // globaler String zum Abspeichern des vom WLAN empfangen Strings in read_wlan

extern const char dev_swname[];
extern const char dev_swversion[];

//Strings im Flash f�r CMD-R�ckmeldungen �ber WLAN
extern const char txtp_cmdend[];
extern const char txtp_errmotor[];
extern const char txtp_sd[];
extern const char txtp_pong[];


#endif /* MAIN_H_ */
