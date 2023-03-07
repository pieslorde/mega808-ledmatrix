#define __AVR_MEGA808__

#include "avr/io.h"
#include "pindefs.h"

void main(void)
{
	VPORTA.DIR = PORTA_OUTPUT_m;
	VPORTD.DIR = PORTD_OUTPUT_m;
	VPORTF.DIR = PORTF_OUTPUT_m;
	while (1){
		VPORTA.OUT = VPORTA.OUT ^ (R_PORTA_m);
	}
}
