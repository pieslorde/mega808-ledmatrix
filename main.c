#include "main.h"

#define SYNC_END_CHARACTER (0xAA)

// Two buffers, first byte indicates offset of buffer being filled
uint8_t recv_buffers[26*2 + 1] __attribute__((aligned (64))); // Aligned to avoid pointer carry
volatile uint8_t buffer_ready = 255; // If the value is 255, neither buffer has data to parse
volatile uint8_t syncing; // If this is zero, we are not synchronizing

// 10 quadruplets of PORTA value, PORTD value, PORTF value, and PWM compare value
// 8 dectuplets for each column
uint8_t portvals[4*10*8];

FUSES = {
	.WDTCFG = FUSE_WDTCFG_DEFAULT,
	.BODCFG = LVL_BODLEVEL7_gc | SAMPFREQ_1KHZ_gc | ACTIVE_ENABLED_gc | SLEEP_ENABLED_gc,
	.OSCCFG = FUSE_OSCLOCK_bm | FREQSEL_20MHZ_gc,
	.SYSCFG0 = CRCSRC_NOCRC_gc | RSTPINCFG_RST_gc,
	.SYSCFG1 = SUT_64MS_gc,
	.APPEND = FUSE_APPEND_DEFAULT,
	.BOOTEND = FUSE_BOOTEND_DEFAULT,
};

ISR(USART1_RXC_vect)
{
	// Simple forwarding
	uint8_t rxdata = USART1.RXDATAL;
	while (USART1.STATUS & USART_DREIF_bm);
	USART1.TXDATAL = rxdata;
}

ISR(TCA0_CMP0_vect, ISR_NAKED)
{
	static uint8_t *portvals_ptr = portvals;
	static uint8_t pwm_column = 8;

	// Save utilized registers (including SREG)
	asm volatile("push r0");
	asm volatile("in r0, %0" :: "I" (_SFR_IO_ADDR(SREG)));
	asm volatile("push r0");
	asm volatile("push r1");
	asm volatile("push r16");
	asm volatile("push r30");
	asm volatile("push r31");

	// Clear interrupt flag
	asm volatile("ldi r30, %0" :: "M" (TCA_SINGLE_CMP0_bm));
	asm volatile("sts %0, r30" :: "m" (TCA0_SINGLE_INTFLAGS));

	// Set Z to portvals_ptr
	asm volatile("lds r30, %0" :: "m" (portvals_ptr));
	asm volatile("lds r31, %0+1" :: "m" (portvals_ptr));

	// Write ports out
	asm volatile("ld r0, Z+");
	asm volatile("out %0, r0" :: "I" (_SFR_IO_ADDR(VPORTA_OUT)));
	asm volatile("ld r0, Z+");
	asm volatile("out %0, r0" :: "I" (_SFR_IO_ADDR(VPORTD_OUT)));
	asm volatile("ld r0, Z+");
	asm volatile("out %0, r0" :: "I" (_SFR_IO_ADDR(VPORTF_OUT)));

	// Get the next compare value and multiply by 173
	asm volatile("ld r0, Z+");
	asm volatile("ldi r16, %0" :: "M" (173));
	asm volatile("mul r0, r16");

	// Write the compare value to TCA0.SINGLE.CMP0
	asm volatile("sts %0, r0" :: "m" (TCA0_SINGLE_CMP0L));
	asm volatile("sts %0, r1" :: "m" (TCA0_SINGLE_CMP0H));

	// If compare value is nonzero, branch to the end
	asm volatile goto("brne %l0" ::: "cc" : _TCA_INT_END);

	// Grab the column counter and decrement
	asm volatile("lds r16, %0" :: "m" (pwm_column));
	asm volatile("dec r16");

	// If we overflowed, we should set the pwm_column to 8
	asm volatile goto("brne %l0" ::: "cc" : _TCA_INT_NO_COLUMN_OVERFLOW);
	asm volatile("ldi r16, %0" :: "M" (8));

_TCA_INT_NO_COLUMN_OVERFLOW:
	// Write back pwm_column
	asm volatile("sts %0, r16" :: "m" (pwm_column));
	// Multiply pwm_column by 40
	asm volatile("ldi r30, %0" :: "M" (40));
	asm volatile("mul r30, r16");
	// Set Z to portvals tail
	asm volatile("ldi r30, lo8(portvals+320)");
	asm volatile("ldi r31, hi8(portvals+320)");
	// And subtract our product from it
	asm volatile("sub r30, r0");
	asm volatile("sbc r31, r1");
	
_TCA_INT_END:
	// Save portvals_ptr
	asm volatile("sts %0, r30" :: "m" (portvals_ptr));
	asm volatile("sts %0+1, r31" :: "m" (portvals_ptr));
	// Restore the utilized registers
	asm volatile("pop r31");
	asm volatile("pop r30");
	asm volatile("pop r16");
	asm volatile("pop r1");
	asm volatile("pop r0");
	asm volatile("out %0, r0" :: "I" (_SFR_IO_ADDR(SREG)));
	asm volatile("pop r0");
	reti();
}

int main(void)
{
	CCP = CCP_IOREG_gc;
	CLKCTRL.MCLKCTRLB = 0; // Let the CPU run at full tilt

	// Outputs are active low, so we set them before enabling
	VPORTA.OUT = PORTA_OUTPUT_m;
	VPORTA.DIR = PORTA_OUTPUT_m;
	VPORTD.OUT = PORTD_OUTPUT_m;
	VPORTD.DIR = PORTD_OUTPUT_m;
	VPORTF.OUT = PORTF_OUTPUT_m;
	VPORTF.DIR = PORTF_OUTPUT_m;

#ifdef __IOTEST_H__
	// This test function does not return
	dim_test();
#endif

	for (uint8_t fill_col = 0; fill_col < 8; fill_col++){
		uint8_t cola_mask = COL_PORTA_m;
		uint8_t cold_mask = COL_PORTD_m;
		uint8_t colf_mask = COL_PORTF_m;
		switch (fill_col){
			case 0:
			cola_mask &= ~COL1_PINA_m;
			break;
			case 1:
			cola_mask &= ~COL2_PINA_m;
			break;
			case 2:
			colf_mask &= ~COL3_PINF_m;
			break;
			case 3:
			colf_mask &= ~COL4_PINF_m;
			break;
			case 4:
			cold_mask &= ~COL5_PIND_m;
			break;
			case 5:
			cold_mask &= ~COL6_PIND_m;
			break;
			case 6:
			cola_mask &= ~COL7_PINA_m;
			break;
			case 7:
			cold_mask &= ~COL8_PIND_m;
			break;
		}
		portvals[fill_col*40 + 0*4 + 0] = cola_mask;
		portvals[fill_col*40 + 0*4 + 1] = cold_mask;
		portvals[fill_col*40 + 0*4 + 2] = colf_mask;
		portvals[fill_col*40 + 0*4 + 3] = 1*(1+fill_col);
		portvals[fill_col*40 + 1*4 + 0] = cola_mask | R1_PINA_m | B1_PINA_m;
		portvals[fill_col*40 + 1*4 + 1] = cold_mask | G1_PIND_m;
		portvals[fill_col*40 + 1*4 + 2] = colf_mask;
		portvals[fill_col*40 + 1*4 + 3] = 5*(1+fill_col);
		portvals[fill_col*40 + 2*4 + 0] = cola_mask | R1_PINA_m | B1_PINA_m | B2_PINA_m;
		portvals[fill_col*40 + 2*4 + 1] = cold_mask | G1_PIND_m | G2_PIND_m | R2_PIND_m;
		portvals[fill_col*40 + 2*4 + 2] = colf_mask;
		portvals[fill_col*40 + 2*4 + 3] = (fill_col == 7 ? 0 : 30*(1+fill_col));
		portvals[fill_col*40 + 3*4 + 0] = cola_mask | ROW_PORTA_m;
		portvals[fill_col*40 + 3*4 + 1] = cold_mask | ROW_PORTD_m;
		portvals[fill_col*40 + 3*4 + 2] = colf_mask;
		portvals[fill_col*40 + 3*4 + 3] = 0;
	}

	recv_buffers[0] = 1+26; // Point the ISR to the end of the first buffer
	
	// Uncomment to allow USART ISR to stomp on PWM ISR
	// CPUINT.LVL1VEC = USART1_RXC_vect_num;

	// USART is 8N1 asynchronous for testing
	// Should be 8N1 synchronous for final
	//USART1.CTRLA = USART_RXCIE_bm;
	USART1.CTRLB = USART_TXEN_bm | USART_RXEN_bm;
	USART1.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_SBMODE_1BIT_gc | USART_CHSIZE_8BIT_gc;
	USART1.BAUD = 87; // 87 = round(64 * 20 MHz / (16 * 921600 Hz))

	// 60 Hz refresh rate, 20 MHz clock, and 240 dimming steps per period
	TCA0.SINGLE.PER = 41519; // 240 * floor(20 MHz / (8 * 60 Hz * 240)) - 1
	TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP0_bm;
	TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;

	sei(); // Holy shit how did I forget about this!!! Took me all day to realize

	while (1){
		
	}
}
