#include "main.h"
#include "iotest.h"

void set_column(uint8_t column)
{
	column = column & 0x7;
	switch (column)
	{
		case 0:
			PORTA.OUTSET = COL_PORTA_m;
			PORTD.OUTSET = COL_PORTD_m;
			PORTF.OUTSET = COL_PORTF_m;
			PORTA.OUTCLR = COL1_PINA_m;
		break;
		case 1:
			PORTA.OUTSET = COL_PORTA_m;
			PORTD.OUTSET = COL_PORTD_m;
			PORTF.OUTSET = COL_PORTF_m;
			PORTA.OUTCLR = COL2_PINA_m;
		break;
		case 2:
			PORTA.OUTSET = COL_PORTA_m;
			PORTD.OUTSET = COL_PORTD_m;
			PORTF.OUTSET = COL_PORTF_m;
			PORTF.OUTCLR = COL3_PINF_m;
		break;
		case 3:
			PORTA.OUTSET = COL_PORTA_m;
			PORTD.OUTSET = COL_PORTD_m;
			PORTF.OUTSET = COL_PORTF_m;
			PORTF.OUTCLR = COL4_PINF_m;
		break;
		case 4:
			PORTA.OUTSET = COL_PORTA_m;
			PORTD.OUTSET = COL_PORTD_m;
			PORTF.OUTSET = COL_PORTF_m;
			PORTD.OUTCLR = COL5_PIND_m;
		break;
		case 5:
			PORTA.OUTSET = COL_PORTA_m;
			PORTD.OUTSET = COL_PORTD_m;
			PORTF.OUTSET = COL_PORTF_m;
			PORTD.OUTCLR = COL6_PIND_m;
		break;
		case 6:
			PORTA.OUTSET = COL_PORTA_m;
			PORTD.OUTSET = COL_PORTD_m;
			PORTF.OUTSET = COL_PORTF_m;
			PORTA.OUTCLR = COL7_PINA_m;
		break;
		case 7:
			PORTA.OUTSET = COL_PORTA_m;
			PORTD.OUTSET = COL_PORTD_m;
			PORTF.OUTSET = COL_PORTF_m;
			PORTD.OUTCLR = COL8_PIND_m;
		break;
		default:
	}
}

void set_rows(uint8_t row)
{
	uint8_t PORTA_OUT_temp = PORTA.OUT;
	uint8_t PORTD_OUT_temp = PORTD.OUT;
	PORTA_OUT_temp |= ROW_PORTA_m;
	PORTD_OUT_temp |= ROW_PORTD_m;
	if (row == 0){
		PORTA_OUT_temp &= ~R1_PINA_m;
		PORTD_OUT_temp &= ~R2_PIND_m;
		PORTD_OUT_temp &= ~R3_PIND_m;
	}
	else if (row == 1){
		PORTD_OUT_temp &= ~G1_PIND_m;
		PORTD_OUT_temp &= ~G2_PIND_m;
		PORTD_OUT_temp &= ~G3_PIND_m;
	}
	else if (row == 2){
		PORTA_OUT_temp &= ~B1_PINA_m;
		PORTA_OUT_temp &= ~B2_PINA_m;
		PORTA_OUT_temp &= ~B3_PINA_m;
	}
	else if (row == 3){
		PORTA_OUT_temp &= ~R1_PINA_m;
		PORTD_OUT_temp &= ~R2_PIND_m;
		PORTD_OUT_temp &= ~R3_PIND_m;
		PORTD_OUT_temp &= ~G1_PIND_m;
		PORTD_OUT_temp &= ~G2_PIND_m;
		PORTD_OUT_temp &= ~G3_PIND_m;
		PORTA_OUT_temp &= ~B1_PINA_m;
		PORTA_OUT_temp &= ~B2_PINA_m;
		PORTA_OUT_temp &= ~B3_PINA_m;
	}
	PORTA.OUT = PORTA_OUT_temp;
	PORTD.OUT = PORTD_OUT_temp;
}

void dim_test(){
	uint8_t col = 0;
	uint8_t duty = 0;
	uint8_t duty_dir = 0;
    while (1){
        // Cycle through every column and row 
        set_column(col++);
        set_rows(3);
        for (uint8_t i = 0; i < duty; i++){
            _delay_us(10);
        }
        set_rows(4);
        for (uint8_t i = 200; i > duty; i--){
            _delay_us(10);
        }
        if (duty_dir){
            if (duty++ == 200){
                duty = 200;
                duty_dir = !duty_dir;
            }
        }
        else{
            if (duty-- == 0){
                duty = 0;
                duty_dir = !duty_dir;
            }
        }
    }
}