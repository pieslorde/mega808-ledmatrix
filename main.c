#include "avr/io.h"
#include "avr/fuse.h"
#include "avr/interrupt.h"

#define F_CPU 20000000
#include "util/delay.h"

#include "pindefs.h"

FUSES = {
	.WDTCFG = FUSE_WDTCFG_DEFAULT,
	.BODCFG = LVL_BODLEVEL7_gc | SAMPFREQ_1KHZ_gc | ACTIVE_ENABLED_gc | SLEEP_ENABLED_gc,
	.OSCCFG = FUSE_OSCLOCK_bm | FREQSEL_20MHZ_gc,
	.SYSCFG0 = CRCSRC_NOCRC_gc | RSTPINCFG_RST_gc,
	.SYSCFG1 = SUT_64MS_gc,
	.APPEND = FUSE_APPEND_DEFAULT,
	.BOOTEND = FUSE_BOOTEND_DEFAULT,
};

ISR(USART1_RXC_vect, ISR_NAKED)
{
	reti();
}

ISR(TCA0_CMP0_vect, ISR_NAKED)
{
	reti();
}

int main(void)
{
	// Uncomment to allow USART ISR to stomp on PWM ISR
	// CPUINT.LVL1VEC = USART1_RXC_vect_num;

	// USART is 8N1 asynchronous for testing
	// Should be 8N1 synchronous for final
	USART1.CTRLA = USART_RXCIE_bm;
	USART1.CTRLB = USART_TXEN_bm | USART_RXEN_bm;
	USART1.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_SBMODE_1BIT_gc | USART_CHSIZE_8BIT_gc;
	USART1.BAUD = 87; // 87 = round(64 * 20 MHz / (16 * 921600 Hz))

	// 60 Hz refresh rate, 20 MHz clock, and 240 dimming steps per period
	TCA0.SINGLE.PER = 41519; // 240 * floor(20 MHz / (8 * 60 Hz * 240)) - 1
	TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP0_bm;
	TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;

	// Columns are active-low, so we need to turn them off first
	VPORTA.OUT = COL_PORTA_m;
	VPORTD.OUT = COL_PORTD_m;
	VPORTF.OUT = COL_PORTF_m;
	VPORTA.DIR = PORTA_OUTPUT_m;
	VPORTD.DIR = PORTD_OUTPUT_m;
	VPORTF.DIR = PORTF_OUTPUT_m;
	while (1){
		_delay_ms(17);
		VPORTA.OUT = VPORTA.OUT ^ (R1_PINA_m);
		_delay_ms(17);
		VPORTA.OUT = VPORTA.OUT ^ (R1_PINA_m | R2_PINA_m);
		_delay_ms(17);
		VPORTA.OUT = VPORTA.OUT ^ (R1_PINA_m);
		_delay_ms(17);
		VPORTA.OUT = VPORTA.OUT ^ (R1_PINA_m | R2_PINA_m | R3_PINA_m);
	}
}
