/*
 * main.h
 *
 *  Created on: 16.11.2010 - 08.05.2015
 *      Author: Michael Brunnbauer
 */

#ifndef MAIN_H_
#define MAIN_H_


//#define LOKBASIS_NO_TEST	// für Testmodus remmen - diverse Sicherheitsabfragen werden für's Testen deaktiviert


// Macros für Bit-Operationen
#define setbit(P,BIT) 	((P) |= (1<<(BIT)))
#define clearbit(P,BIT)	((P) &= ~(1<<(BIT)))

#define UART_MAXSTRLEN 32		// maximal erlaubte Länge eines Befehls "<*>" incl. <>

#define ALIVE_INTERVAL 5		//Prüfintervall = 5 Sekunden! -> Lok stoppt nach 5sek herrenloser Fahrt

// Statusdefinitionen für Interrupts und Main-Schleife
#define STATE_1X_PRO_SEK 1		// Timer meldet sich 1x pro Sekunde
#define STATE_5X_PRO_SEK 2		// Timer meldet sich 5x pro Sekunde
#define STATE_3 4				//
#define STATE_4 8				//
#define STATE_5 16				//
#define STATE_6 32				//
#define STATE_7 64				//
#define STATE_8 128				//


// definitionen für Steuerung
#define RICHTUNG_VW	1		// Richtung: vorwärts = 1
#define RICHTUNG_RW	0		// Richtung: rückwärts = 0

// extern verwendete Variablen
extern volatile unsigned char state;		// Status-Variable für Interrupts und Main-Schleife
extern unsigned int speed;					// Geschwindigkeitswert der an die H-Brücke ausgegeben wird (momentaner Wert)
extern unsigned int speed_soll;				// Geschwindigkeitsvorgabe vom Steuerungsgerät (Wert, der erreicht werden soll)
extern unsigned char richtung;
extern unsigned char richtung_soll;
extern unsigned char speedstep_korrektur;
extern volatile unsigned char motor_reg;	// Variable für Motor-Regelung ein/aus (auch für isr verwendet)
extern uint8_t motorerror;					// Errorcode von Motorcontroller: 0 = kein Error
extern unsigned char alivecount;			// zählt die empfangen Meldungen der Gegenstelle (wird zyklisch ausgewertet)
extern volatile char alone;
extern char wlan_string[UART_MAXSTRLEN+1];  // globaler String zum Abspeichern des vom WLAN empfangen Strings in read_wlan

extern const char dev_swname[];
extern const char dev_swversion[];

//Strings im Flash für CMD-Rückmeldungen über WLAN
extern const char txtp_cmdend[];
extern const char txtp_errmotor[];
extern const char txtp_sd[];
extern const char txtp_pong[];


#endif /* MAIN_H_ */
