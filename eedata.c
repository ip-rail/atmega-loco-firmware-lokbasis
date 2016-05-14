/*
 * eedata.c
 *
 *  Created on: 04.06.2015
 *      Author: Michael Brunnbauer
 */



#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <string.h>		//memset
#include "lokbasis_hwdef.h"		// Hardware-Definitionen für die verschiedenen Boards
#include "main.h"
#include "eedata.h"
#include "servo.h"

/*
// EEPROM data structure - nur zu Definitionszwecken
typedef struct {
  uint32_t magic_code;	//	0xAABBCCDD
  uint16_t eedata_version;	// fortlaufende Nummer bei Änderung
  //--- HW-Config
  uint8_t  adc_used;	//ADC0-7 bitmask: 0=unused, 1=used
  uint8_t  motor_pwmf;
  uint8_t  motor_config;	// Konfig H-Brücken Stdwert = 8 für 1 H-Brücke (25 für 2 H-Brücken)
  uint8_t  alivechecksecs; 	Einstellung für den Alive-check [Sekunden, die die Lok ohne neue Befehle weiterfahren darf] 0 -> keine Prüfung
  uint8_t  servo_mode;	// 0: für jedes Servosignal wird ein GPIO verwendet (vordefinieren welche!). 1: alle Signale nur an 1 GPIO ausgeben (Port,Pin mit Index 0), an dem ein Kreiszähler-IC hängt
  uint8_t  servo_count;	// wieviele Servosignale ausgeben (max. 6-8?)
  uint8_t servoPort[SERVOCOUNTMAX];	//Port-Adresse und Pin-Nummer der GPIOs, die für Servos verwendet werden (verwendet werden Index 0 bis SERVOCOUNTMAX-1)
  uint8_t servoPin[SERVOCOUNTMAX];	//Port und Pin gehören je Index zusammen
  // TODO Servo Anschlagskorrektur top/bottom ?? oder nur im Controller?

  uint8_t  gpios_b;		// GPIOs Port B: 1: verwendet, 0: nicht verwendet
  uint8_t  gpios_d;		//
  uint8_t  gpios_e;		//
  uint8_t  gpios_g;		//
  // gpios müssen noch mit den vordefinierten anders-verwendeten Pins ver-UNDet werden! und verwendete Servo-Pins berücksichtigen, dann stehen sie als GPIO auch nicht mehr zur Verfügung!
  // hier wird nur gespeichert, welche freien GPIOs der User verwenden will - vordefiniert ist bereits, was programmintern anders verwendet wird und daher sowieso nicht zur Verfügung steht

  // String data
  char lok_name[41];		// Standardwert "Lok X"
  char owner_name[41];		// Standardwert "TheOwner"

} EEPROMdata; */



void eeprom_checkversion()
{
	if (!(eeprom_read_dword((const uint32_t *)EEDATA_ADR_START) == (uint32_t)EE_MAGIC_CODE))	{ init_eeprom(); }	// magic code not yet set -> init
	else
	{
		uint16_t ee_saveddata_ver = eeprom_read_word((const uint16_t *)EEDATA_ADR_VERSION);
		if (ee_saveddata_ver < EEDATA_VERSION) { eeprom_updatesys(ee_saveddata_ver); }	// Update notwendig
	}
}

void eeprom_updatesys(uint16_t oldversion)
{
	// TODO: Data structure updates
	switch (oldversion)
		{
			case 1:	// Update von Version 1 auf 2
			case 2:	// Update von Version 2 auf 3
			case 3: // Update von Version 3 auf 4
				init_eeprom();	// ein letztes Mal drüberklatschen (damit wird auch die version upgedatet)
			break;
		}
}



//EEData mit Defaultwerten befüllen
void init_eeprom()
{
	char datatxt[EEDATA_MAXSTRLEN];

	//EEData version 3
	eeprom_update_dword((uint32_t *)EEDATA_ADR_START, (uint32_t)EE_MAGIC_CODE);
	//eeprom_update_word((uint16_t *)EEDATA_ADR_VERSION, (uint16_t)EEDATA_VERSION);	// TODO: error: eeData version wird so nicht geschrieben!!!!
	eeprom_update_word((uint16_t *)4, 4);	// eeData version = 4 - so wird es geschrieben!!!!!!!!
	eeprom_update_byte((uint8_t *)EEDATA_ADR_ADCUSED, 1);	// only use ADC0
	eeprom_update_byte((uint8_t *)EEDATA_ADR_MOTOR_PWMF, (uint8_t)MOTOR_PWMF_STD);
	eeprom_update_byte((uint8_t *)EEDATA_ADR_MOTOR_CONFIG, (uint8_t)MOTOR_CONFIG_1HB);
	eeprom_update_byte((uint8_t *)EEDATA_ADR_ALIVE_SECS, (uint8_t)ALIVE_INTERVAL);
	eeprom_update_byte((uint8_t *)EEDATA_ADR_SERVO_MODE, 0);	// a GPIO for each Servo
	//eeprom_update_byte((uint8_t *)EEDATA_ADR_SERVO_COUNT, 0);	// no Servos used -> servo port,pin data doesn't matter!
	eeprom_update_byte((uint8_t *)EEDATA_ADR_SERVO_COUNT, 2);// TODO: nur Testdaten, wieder entfernen!

	servoPort[0] = 'B';	// TODO: nur test
	servoPin[0]  = PB0;	// TODO: nur test
	servoPort[1] = 'B';	// TODO: nur test
	servoPin[1]  = PB1;	// TODO: nur test

	eeprom_update_ServoGPIO(servoPort, servoPin);	// TODO: nur test, wieder entfernen!

	eeprom_update_byte((uint8_t *)EEDATA_ADR_GPIOS_B, 0);	// Defaultwert 0: keine GPIOs verwenden
	eeprom_update_byte((uint8_t *)EEDATA_ADR_GPIOS_D, 0);
	eeprom_update_byte((uint8_t *)EEDATA_ADR_GPIOS_E, 0);
	eeprom_update_byte((uint8_t *)EEDATA_ADR_GPIOS_G, 0);

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
		eeprom_update_block((const void *)data, (void *)EEDATA_ADR_OWNERNAME, datasize+1);	// datasize+1 -> abschließendes 0-Byte mitkopieren!! (ob danach noch Mist steht, falls der String nicht die volle reservierte Länge ausf�llt, ist egal
	}
}

void eeprom_get_oname(char *name)
{
	eeprom_read_block((void*)name, (void*)EEDATA_ADR_OWNERNAME, EEDATA_MAXSTRLEN);
}

void eeprom_update_lname(const char *data)
{
	uint16_t datasize = strlen(data);

	if (datasize < (EEDATA_MAXSTRLEN))
	{
		eeprom_update_block((const void *)data, (void *)EEDATA_ADR_LOKNAME, datasize+1);	// datasize+1 -> abschließendes 0-Byte mitkopieren!! (ob danach noch Mist steht, falls der String nicht die volle reservierte Länge ausfüllt, ist egal
	}
}

void eeprom_get_lname(char *name)
{
	eeprom_read_block((void*)name, (void*)EEDATA_ADR_LOKNAME, EEDATA_MAXSTRLEN);
}

void eeprom_getServoMode()
{
	servo_mode = eeprom_read_byte((const uint8_t *)EEDATA_ADR_SERVO_MODE);
}

void eeprom_update_ServoMode(const uint8_t mode)
{
	eeprom_update_byte((uint8_t *)EEDATA_ADR_SERVO_MODE, mode);
}

void eeprom_getServoCount()
{
	servo_count = eeprom_read_byte((const uint8_t *)EEDATA_ADR_SERVO_COUNT);
}

void eeprom_update_ServoCount(const uint8_t count)
{
	eeprom_update_byte((uint8_t *)EEDATA_ADR_SERVO_COUNT, count);
}

void eeprom_getServoGPIO()
{
	eeprom_read_block((void *)&servoPort, (const void *)EEDATA_ADR_SERVO_PORT, SERVOCOUNTMAX);
	eeprom_read_block((void *)&servoPin, (const void *)EEDATA_ADR_SERVO_PIN, SERVOCOUNTMAX);

	// Info: servoOutput[] wird im initServo() nach dem Aufruf von eeprom_getServoGPIO() generiert
}


void eeprom_update_ServoGPIO(const void *ports, const void *pins)	// ports und pins array
{
	eeprom_update_block((const void *)ports, (void *)EEDATA_ADR_SERVO_PORT, SERVOCOUNTMAX);
	eeprom_update_block((const void *)pins, (void *)EEDATA_ADR_SERVO_PIN, SERVOCOUNTMAX);
}


uint8_t eeprom_getADCGPIO()
{
	return eeprom_read_byte((const uint8_t *)EEDATA_ADR_ADCUSED);
}

void eeprom_update_ADCGPIO(const uint8_t adcmask)
{
	eeprom_update_byte((uint8_t *)EEDATA_ADR_ADCUSED, adcmask);
}

uint8_t eeprom_getMotorPWMf()
{
	return eeprom_read_byte((const uint8_t *)EEDATA_ADR_MOTOR_PWMF);
}

void eeprom_update_MotorPWMf(const uint8_t pwmf)
{
	eeprom_update_byte((uint8_t *)EEDATA_ADR_MOTOR_PWMF, pwmf);
}


uint8_t eeprom_getMotorConfig()
{
	return eeprom_read_byte((const uint8_t *)EEDATA_ADR_MOTOR_CONFIG);
}

void eeprom_update_MotorConfig(const uint8_t mconfig)
{
	eeprom_update_byte((uint8_t *)EEDATA_ADR_MOTOR_CONFIG, mconfig);
}




uint8_t eeprom_getAliveCheckSecs()
{
	return eeprom_read_byte((const uint8_t *)EEDATA_ADR_ALIVE_SECS);
}

void eeprom_update_AliveCheckSecs(const uint8_t secs)
{
	eeprom_update_byte((uint8_t *)EEDATA_ADR_ALIVE_SECS, secs);
}


uint8_t eeprom_getGPIO(char port)	// TODO: ACHTUNG: muss Großbuchstabe sein!! -> bei Befehlsauswertung beachten, muwandeln!!
{
	uint8_t value = 0;

	switch (port)
	{
	case 'B':
		value = eeprom_read_byte((const uint8_t *)EEDATA_ADR_GPIOS_B);
		break;

	case 'D':
		value = eeprom_read_byte((const uint8_t *)EEDATA_ADR_GPIOS_D);
		break;

	case 'E':
		value = eeprom_read_byte((const uint8_t *)EEDATA_ADR_GPIOS_E);
		break;

	case 'G':
		value = eeprom_read_byte((const uint8_t *)EEDATA_ADR_GPIOS_G);
		break;
	}

	return value;
}

void eeprom_update_GPIO(uint8_t maskB, uint8_t maskD, uint8_t maskE, uint8_t maskG)
{
	eeprom_update_byte((uint8_t *)EEDATA_ADR_GPIOS_B, maskB);
	eeprom_update_byte((uint8_t *)EEDATA_ADR_GPIOS_D, maskD);
	eeprom_update_byte((uint8_t *)EEDATA_ADR_GPIOS_E, maskE);
	eeprom_update_byte((uint8_t *)EEDATA_ADR_GPIOS_G, maskG);
}

