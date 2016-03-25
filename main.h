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

#define UART_MAXSTRLEN 64		// maximal erlaubte Länge eines Befehls "<*>" incl. <>

#define ALIVE_INTERVAL 3		//Prüfintervall = 3 Sekunden! -> Lok stoppt nach 3sek herrenloser Fahrt

// Statusdefinitionen für Interrupts und Main-Schleife
#define STATE_1X_PRO_SEK 1		// Timer meldet sich 1x pro Sekunde
#define STATE_5X_PRO_SEK 2		// Timer meldet sich 5x pro Sekunde
#define STATE_ADC_CHECK  4		// ADC-Daten bereit zum Auswerten
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
// extern volatile unsigned char motor_reg;	// Variable für Motor-Regelung ein/aus (auch für isr verwendet)
extern uint8_t motorerror;					// Errorcode von Motorcontroller: 0 = kein Error
extern uint8_t motor_pwmf;					// Auswahl der Motor-PWM-Frequenz
extern uint8_t motor_cfg;					// Konfig H-Brücken
extern uint8_t alivecount;					// zählt die empfangen Meldungen der Gegenstelle (wird zyklisch ausgewertet)
extern uint8_t maxalivesecs;				// Einstellung für den Alive-check
extern char wlan_string[UART_MAXSTRLEN+1];  // globaler String zum Abspeichern des vom WLAN empfangen Strings in read_wlan

extern volatile uint8_t adcchannel;		// aktueller ADC channel 0-7
extern uint8_t adc_mask;					// die benützten ADC Channels (siehe auch: adc_used in eedata.c), kein volatile, da sich der Wert im Betrieb nicht ändert
extern volatile uint8_t adcreadcount;		// counter für Lesevorgänge pro ADC channel
extern unsigned int adcvalue[8];
extern volatile unsigned int adcvalue_work;

extern const char dev_swname[];
extern const char dev_swversion[];

//Strings im Flash für CMD-Rückmeldungen über WLAN
extern const char txtp_cmdend[];
extern const char txtp_cmdtrenn[];
extern const char txtp_errmotor[];
extern const char txtp_sd[];
extern const char txtp_iam[];
extern const char txtp_pong[];
extern const char txtp_default_lok_name[];
extern const char txtp_default_owner_name[];
extern const char txtp_hwi[];
extern const char txtp_cmd_servoi[];
extern const char txtp_cmd_ui[];
extern const char txtp_cmd_fpwmi[];
extern const char txtp_cmd_mcfgi[];
extern const char txtp_cmd_log[];
extern const char txtp_cmd_ntypi[];
extern const char txtp_cmd_onamei[];
extern const char txtp_cmd_alivei[];

//Befehle im Flash
extern const char txtp_cmd_stop[];
extern const char txtp_cmd_off[];
extern const char txtp_cmd_stopall[];
extern const char txtp_cmd_richtung[];
extern const char txtp_cmddata_vw[];
extern const char txtp_cmddata_rw[];
extern const char txtp_cmd_sd[];
extern const char txtp_cmd_ping[];
extern const char txtp_cmd_l1[];
extern const char txtp_cmd_l0[];
extern const char txtp_cmd_reset[];
extern const char txtp_cmd_onameset[];
extern const char txtp_cmd_nameset[];
extern const char txtp_cmddata_start[];
extern const char txtp_cmddata_add[];
extern const char txtp_cmddata_end[];
extern const char txtp_cmd_hwget[];
extern const char txtp_cmd_servoget[];
extern const char txtp_cmd_servoset[];
extern const char txtp_cmd_gpioc[];
extern const char txtp_cmd_fpwmset[];
extern const char txtp_cmd_fpwmget[];
extern const char txtp_cmd_alive[];
extern const char txtp_cmd_mcfgget[];
extern const char txtp_cmd_mcfgset[];
extern const char txtp_cmd_nameget[];
extern const char txtp_cmd_onameget[];
extern const char txtp_cmd_aliveget[];
extern const char txtp_cmd_aliveset[];

#endif /* MAIN_H_ */
