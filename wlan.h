/*
 * wlan.h
 *
 *  Created on: 29.01.2011 - 02.05.2015
 *      Author: Michael Brunnbauer
 */

#ifndef WLAN_H_
#define WLAN_H_

extern void check_wlan_cmd();
//extern void read_wlan(uint8_t wait_sek);


// Werte f�r cmdstate - Status des aktuellen WLAN cmd strings
#define WLANCMD_NONE 		0		// noch nichts von einem cmd empfangen
#define WLANCMD_STARTED		1		// Startzeichen wurde entdeckt, folgede Zeichen �bernehmen


#endif /* WLAN_H_ */
