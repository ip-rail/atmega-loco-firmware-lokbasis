/*
 * eedata.h
 *
 *  Created on: 04.06.2015
 *      Author: Michael Brunnbauer
 */


#ifndef EEDATA_H_
#define EEDATA_H_


#define EEDATA_MAXSTRLEN 41		// max. Länge für Lok-Namen, Owner-Namen (inkl. Leerzeichen)

#define EEDATA_ADR_START 			0		// Startadresse der Daten im EEPROM
#define EEDATA_ADR_VERSION			EEDATA_ADR_START + 4
#define EEDATA_ADR_ADCUSED			EEDATA_ADR_VERSION + 2
#define EEDATA_ADR_SERVO_MODE		EEDATA_ADR_ADCUSED + 1
#define EEDATA_ADR_SERVO_COUNT		EEDATA_ADR_SERVO_MODE + 1
#define EEDATA_ADR_SERVO_PORT		EEDATA_ADR_SERVO_COUNT + 1
#define EEDATA_ADR_SERVO_PIN 		EEDATA_ADR_SERVO_PORT + SERVOCOUNTMAX
#define EEDATA_ADR_LOKNAME			EEDATA_ADR_SERVO_PIN + SERVOCOUNTMAX
#define EEDATA_ADR_OWNERNAME		EEDATA_ADR_LOKNAME + EEDATA_MAXSTRLEN

#define EE_MAGIC_CODE	0xAABBCCDD	// Kennzeichnung, dass EEPROM bereits gültige Daten enthält
#define EEDATA_VERSION	1			// aktuelle Version der Daten(struktur)


extern void eeprom_checkversion();
extern void init_eeprom();
extern void eeprom_update_oname(const char *data);
extern void eeprom_update_lname(const char *data);

#endif /* EEDATA_H_ */
