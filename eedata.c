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



// EEPROM data structure - nur zu Definitionszwecken
typedef struct {
  uint32_t magic_code;	//	0xAABBCCDD
  uint16_t eedata_version;	// fortlaufende Nummer bei Änderung
  //--- HW-Config
  uint8_t  adc_used;	//ADC0-7: 0=unused, 1=used
  //uint8_t  servo_mode;	// 0: für jedes Servosignal wird ein GPIO verwendet (vordefinieren welche!). 1: alle Signale nur an 1 GPIO, an dem ein Kreiszähler-IC hängt
  //uint8_t  servo_count;	// wieviele Servosignale ausgeben (max. 8?)
  //uint8_t  gpios_b;		// GPIOs Port B: 1: verwendet, 0: nicht verwendet
  //uint8_t  gpios_d;		//
  //uint8_t  gpios_e;		//
  //uint8_t  gpios_g;		//
  // gpios müssen noch mit den vordefinierten anders-verwendeten Pins ver-UNDet werden!
  // hier wird nur gespeichert, welche freien GPIOs der User verwenden will - vordefiniert ist bereits, was programmintern anders verwendet wird und daher sowieso nicht zur Verfüngung steht


  // String data
  char lok_name[41];		// Standardwert "Lok X"
  char owner_name[41];		// Standardwert "TheOwner"

} EEPROMdata;



void eeprom_checkversion()
{
	if (!(eeprom_read_dword((const uint32_t *)EEDATA_START) == (uint32_t)EE_MAGIC_CODE))	{ init_eeprom(); }	// magic code not yet set -> init
	else
	{
		uint16_t ee_saveddata_ver = eeprom_read_word((const uint16_t *)EEDATA_START + 4);
		if (ee_saveddata_ver < EEDATA_VERSION) // Update notwendig
		{
			// switch
		}


	}

}


//EEData mit Defaultwerten befüllen
void init_eeprom()
{
	char datatxt[EEDATA_MAXSTRLEN];

	//EEData version 0
	eeprom_update_dword((uint32_t *)EEDATA_START, (uint32_t)EE_MAGIC_CODE);
	eeprom_update_word((uint16_t *)EEDATA_START + 4, 1);	// Data version 1
	eeprom_update_byte((uint8_t *)EEDATA_START + 6, 1);	// only use ADC0

	memset(datatxt, 0, EEDATA_MAXSTRLEN);	// string leeren
	strncpy_P(datatxt, txtp_default_lok_name, 5);
	eeprom_update_block((const void *)datatxt, (void *)EEDATA_START + 7, strlen(datatxt));

	memset(datatxt, 0, EEDATA_MAXSTRLEN);
	strncpy_P(datatxt, txtp_default_owner_name, 8);
	eeprom_update_block((const void *)datatxt, (void *)EEDATA_START + 48, strlen(datatxt));

}


void eeprom_update_oname(const char * data)
{
	uint16_t datasize = strlen(data);

	if (datasize < (EEDATA_MAXSTRLEN-1))
	{
		eeprom_update_block((const void *)data, (void *)EEDATA_START + 48, datasize);
	}

}






