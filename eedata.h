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
#define EEDATA_ADR_MOTOR_PWMF		EEDATA_ADR_ADCUSED + 1
#define EEDATA_ADR_MOTOR_CONFIG		EEDATA_ADR_MOTOR_PWMF + 1

#define EEDATA_ADR_ALIVE_SECS		EEDATA_ADR_MOTOR_CONFIG + 1

#define EEDATA_ADR_SERVO_MODE		EEDATA_ADR_ALIVE_SECS + 1
#define EEDATA_ADR_SERVO_COUNT		EEDATA_ADR_SERVO_MODE + 1
#define EEDATA_ADR_SERVO_PORT		EEDATA_ADR_SERVO_COUNT + 1
#define EEDATA_ADR_SERVO_PIN 		EEDATA_ADR_SERVO_PORT + SERVOCOUNTMAX

#define EEDATA_ADR_GPIOS_B			EEDATA_ADR_SERVO_PIN + SERVOCOUNTMAX
#define EEDATA_ADR_GPIOS_D			EEDATA_ADR_GPIOS_B + 1
#define EEDATA_ADR_GPIOS_E			EEDATA_ADR_GPIOS_D + 1
#define EEDATA_ADR_GPIOS_G			EEDATA_ADR_GPIOS_E + 1
#define EEDATA_ADR_LOKNAME			EEDATA_ADR_GPIOS_G + 1
#define EEDATA_ADR_OWNERNAME		EEDATA_ADR_LOKNAME + EEDATA_MAXSTRLEN

#define EE_MAGIC_CODE	0xAABBCCDD	// Kennzeichnung, dass EEPROM bereits gültige Daten enthält
#define EEDATA_VERSION	4			// aktuelle Version der Daten(Struktur)


extern void eeprom_checkversion();
extern void eeprom_updatesys(uint16_t oldversion);
extern void init_eeprom();
extern void eeprom_update_oname(const char *data);
extern void eeprom_get_oname(char *name);
extern void eeprom_update_lname(const char *data);
extern void eeprom_get_lname(char *name);
extern void eeprom_getServoMode();
extern void eeprom_update_ServoMode(const uint8_t mode);
extern void eeprom_getServoCount();
extern void eeprom_update_ServoCount(const uint8_t count);
extern void eeprom_getServoGPIO();
extern void eeprom_update_ServoGPIO(const void *ports, const void *pins);
extern uint8_t  eeprom_getADCGPIO();
extern void  eeprom_update_ADCGPIO(const uint8_t adcmask);
extern uint8_t eeprom_getMotorPWMf();
extern void eeprom_update_MotorPWMf(const uint8_t pwmf);
extern uint8_t eeprom_getMotorConfig();
extern void eeprom_update_MotorConfig(const uint8_t mconfig);
extern uint8_t eeprom_getAliveCheckSecs();
extern void eeprom_update_AliveCheckSecs(const uint8_t secs);
extern uint8_t eeprom_getGPIO(char port);
extern void eeprom_update_GPIO(uint8_t maskB, uint8_t maskD, uint8_t maskE, uint8_t maskG);

#endif /* EEDATA_H_ */
