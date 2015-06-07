/*
 * eedata.h
 *
 *  Created on: 04.06.2015
 *      Author: Michael Brunnbauer
 */


#ifndef EEDATA_H_
#define EEDATA_H_


#define EEDATA_MAXSTRLEN 41		// max. Länge für Lok-Namen, Owner-Namen (inkl. Leerzeichen)

#define EEDATA_START 	0			// Startadresse der Daten im EEPROM
#define EE_MAGIC_CODE	0xAABBCCDD	// Kennzeichnung, dass EEPROM bereits gültige Daten enthält
#define EEDATA_VERSION	1			// aktuelle Version der Daten(struktur)


extern void eeprom_checkversion();
extern void init_eeprom();
extern void eeprom_update_oname(const char * data);

#endif /* EEDATA_H_ */
