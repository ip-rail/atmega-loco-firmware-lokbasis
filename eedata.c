/*
 * eedata.c
 *
 *  Created on: 04.06.2015
 *      Author: Michael Brunnbauer
 */



#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <string.h>		//memset
#include "main.h"
#include "eedata.h"
#include "servo.h"


// EEPROM data structure - nur zu Definitionszwecken
typedef struct {
  uint32_t magic_code;	//	0xAABBCCDD
  uint16_t eedata_version;	// fortlaufende Nummer bei Änderung
  //--- HW-Config
  uint8_t  adc_used;	//ADC0-7 bitmask: 0=unused, 1=used
  uint8_t  servo_mode;	// 0: für jedes Servosignal wird ein GPIO verwendet (vordefinieren welche!). 1: alle Signale nur an 1 GPIO ausgeben (Port,Pin mit Index 0), an dem ein Kreiszähler-IC hängt
  uint8_t  servo_count;	// wieviele Servosignale ausgeben (max. 6-8?)
  uint8_t servoPort[SERVOCOUNTMAX];	//Port und Pin der GPIOs, die für Servos verwendet werden (verwendet werden Index 0 bis SERVOCOUNTMAX-1)
  uint8_t servoPin[SERVOCOUNTMAX];	//Port und Pin gehören je Index zusammen
  // TODO Servo Anschlagskorrektur top/bottom ?? oder nur im Controller?

  //uint8_t  gpios_b;		// GPIOs Port B: 1: verwendet, 0: nicht verwendet
  //uint8_t  gpios_d;		//
  //uint8_t  gpios_e;		//
  //uint8_t  gpios_g;		//
  // gpios müssen noch mit den vordefinierten anders-verwendeten Pins ver-UNDet werden! und verwendete Servo-Pins berücksichtigen, dann stehen sie als GPIO auch nicht mehr zur Verfügung!
  // hier wird nur gespeichert, welche freien GPIOs der User verwenden will - vordefiniert ist bereits, was programmintern anders verwendet wird und daher sowieso nicht zur Verfügung steht

  // String data
  char lok_name[41];		// Standardwert "Lok X"
  char owner_name[41];		// Standardwert "TheOwner"

} EEPROMdata;



void eeprom_checkversion()
{
	if (!(eeprom_read_dword((const uint32_t *)EEDATA_ADR_START) == (uint32_t)EE_MAGIC_CODE))	{ init_eeprom(); }	// magic code not yet set -> init
	else
	{
		uint16_t ee_saveddata_ver = eeprom_read_word((const uint16_t *)EEDATA_ADR_VERSION);
		if (ee_saveddata_ver < EEDATA_VERSION) // Update notwendig
		{
			// switch	// TODO: Data structure updates
		}

	}

}


//EEData mit Defaultwerten befüllen
void init_eeprom()
{
	char datatxt[EEDATA_MAXSTRLEN];

	//EEData version 0
	eeprom_update_dword((uint32_t *)EEDATA_ADR_START, (uint32_t)EE_MAGIC_CODE);
	eeprom_update_word((uint16_t *)EEDATA_ADR_VERSION, 1);	// Data version 1
	eeprom_update_byte((uint8_t *)EEDATA_ADR_ADCUSED, 1);	// only use ADC0
	eeprom_update_byte((uint8_t *)EEDATA_ADR_SERVO_MODE, 0);	// a GPIO for each Servo
	eeprom_update_byte((uint8_t *)EEDATA_ADR_SERVO_COUNT, 0);	// no Servos used -> serov port,pin data doesn't matter!

	memset(datatxt, 0, EEDATA_MAXSTRLEN);	// string leeren
	strncpy_P(datatxt, txtp_default_lok_name, 5);
	eeprom_update_block((const void *)datatxt, (void *)EEDATA_ADR_LOKNAME, EEDATA_MAXSTRLEN);

	memset(datatxt, 0, EEDATA_MAXSTRLEN);
	strncpy_P(datatxt, txtp_default_owner_name, 8);
	eeprom_update_block((const void *)datatxt, (void *)EEDATA_ADR_OWNERNAME, EEDATA_MAXSTRLEN);

}


void eeprom_update_oname(const char *data)
{
	uint16_t datasize = strlen(data);

	if (datasize < (EEDATA_MAXSTRLEN))
	{
		eeprom_update_block((const void *)data, (void *)EEDATA_ADR_OWNERNAME, datasize+1);	// datasize+1 -> abschließendes 0-Byte mitkopieren!! (ob danach noch Mist steht, falls der String nicht die volle reservierte Länge ausfüllt, ist egal
	}
}

void eeprom_update_lname(const char *data)
{
	uint16_t datasize = strlen(data);

	if (datasize < (EEDATA_MAXSTRLEN))
	{
		eeprom_update_block((const void *)data, (void *)EEDATA_ADR_LOKNAME, datasize+1);	// datasize+1 -> abschließendes 0-Byte mitkopieren!! (ob danach noch Mist steht, falls der String nicht die volle reservierte Länge ausfüllt, ist egal
	}
}




