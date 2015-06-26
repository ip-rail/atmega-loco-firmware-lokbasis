/*
 * adc.h
 *
 *  Created on: 24.06.2015
 *      Author: Michael Brunnbauer
 */

#ifndef ADC_H_
#define ADC_H_

#define MUX5 3		// MUX5 = ADCSRB, bit 3 (von bit0 weg gezählt!) - scheint nicht definiert zu sein

extern void init_adc();
extern void check_adc();
extern void adc_msg_all(char *txtbuffer);


#endif /* ADC_H_ */
