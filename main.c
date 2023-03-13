#include "main.h"

#define SYNC_END_CHARACTER (0xAA)
#define FRAME_LENGTH (49) // 24 LEDs with 2-byte values
#define BUFFER_LENGTH (FRAME_LENGTH + 4) // Frame plus 4 state variables
#define SYNCING_OFFSET (FRAME_LENGTH + 3)
#define RECV_IDX_OFFSET (FRAME_LENGTH + 2)
#define SAVING_OFFSET (FRAME_LENGTH + 1)
#define BUFFER_READY_OFFSET (FRAME_LENGTH + 0)

#define PORTVALS_LENGTH (4*10*8*2)

// Buffer of USART data terminated with buffer_ready, saving, recv_idx, and syncing
volatile uint8_t recv_buffer[BUFFER_LENGTH] __attribute__((aligned (64)));

// 10 quadruplets of PORTA value, PORTD value, PORTF value, and PWM compare value
// 8 dectuplets for each column
// Two buffers terminated by an offset from the tail
volatile uint8_t portvals[PORTVALS_LENGTH];
volatile uint8_t *portvals_tail_ptr = &portvals[4*10*8];

// Lookup tables for column masks
static const uint8_t colmasksa[8] = {~COL1_PINA_m, ~COL2_PINA_m, ~0, ~0, ~0, ~0, ~COL7_PINA_m, ~0};
static const uint8_t colmasksd[8] = {~0, ~0, ~0, ~0, ~COL5_PIND_m, (uint8_t)(~COL6_PIND_m), ~0, ~COL8_PIND_m};
static const uint8_t colmasksf[8] = {~0, ~0, ~COL3_PINF_m, ~COL4_PINF_m, ~0, ~0, ~0, ~0};

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
	// Save SREG and r16
	asm volatile("push r16");
	asm volatile("in r16, %0" :: "I" (_SFR_IO_ADDR(SREG)));
	asm volatile("push r16");

	// Read the new data into r16
	asm volatile("lds r16, %0" :: "m" (USART1_RXDATAL));

	// Save the rest
	asm volatile("push r17");
	asm volatile("push ZL");
	asm volatile("push ZH");

	// Load base of recv_buffer to Z
	asm volatile("ldi ZL, lo8(%0)" :: "m" (recv_buffer));
	asm volatile("ldi ZH, hi8(%0)" :: "m" (recv_buffer));

	// Load syncing into r17 and check if we are syncing
	asm volatile("ldd r17, Z+%0" :: "I" (SYNCING_OFFSET));
	asm volatile("tst r17");
	asm volatile goto("brne %l0" ::: "cc" : _USART_INT_SYNCING_IS_TRUE);

	// We are not syncing so we proceed by loading the receive index to r17
	asm volatile("ldd r17, Z+%0" :: "I" (RECV_IDX_OFFSET));

	// Check if we are reading a frame ID
	asm volatile("tst r17");
	asm volatile goto("breq %l0" ::: "cc" : _USART_INT_RECEIVED_FRAME_ID);

_USART_INT_RECEIVED_REGULAR_BYTE:
	// We are reading a regular character and need to see if we have to save it
	asm volatile("push r18");
	asm volatile("ldd r18, Z+%0" :: "I" (SAVING_OFFSET));

	// If we are not saving, skip over the next bit
	asm volatile("tst r18");
	asm volatile goto("breq %l0" ::: "cc" : _USART_INT_SKIP_SAVING);

	// Add the offset to Z, write the buffer, and restore X
	asm volatile("add ZL, r17");
	asm volatile("st Z, r16");
	asm volatile("sub ZL, r17");

	// If we just finished filling the buffer, we should handle it
	asm volatile("mov r18, r17");
	asm volatile("cpi r18, %0" :: "M" (FRAME_LENGTH-1));
	asm volatile goto("breq %l0" ::: "cc" : _USART_INT_SET_BUFFER_READY);

_USART_INT_SKIP_SAVING:
	// Increment receive index and load compare value to r18
	asm volatile("inc r17");
	asm volatile("ldi r18, %0" :: "M" (FRAME_LENGTH));

	// If they are equal, clear r17
	asm volatile("cpse r18, r17");
	asm volatile goto("rjmp %l0" ::: "cc" : _USART_INT_BUFFER_READY_DONE);
	asm volatile("clr r17");

_USART_INT_BUFFER_READY_DONE:
	// Restore r18 and write r17 back to memory
	asm volatile("pop r18");
	asm volatile("std Z+%0, r17" :: "I" (RECV_IDX_OFFSET));

_USART_INT_SEND_CHAR:
	// While DREIF is 0, busy wait
	asm volatile("lds r17, %0" :: "m" (USART1_STATUS));
	asm volatile("sbrs r17, %0" :: "M" (USART_DREIF_bp)); // Continue if bit is set
	asm volatile goto("rjmp %l0" ::: "cc" : _USART_INT_SEND_CHAR);

	// Write r16 to TXDATAL
	asm volatile("sts %0, r16" :: "m" (USART1_TXDATAL));

	// Restore all modified registers and return
	asm volatile("pop ZH");
	asm volatile("pop ZL");
	asm volatile("pop r17");
	asm volatile("pop r16");
	asm volatile("out %0, r16" :: "I" (_SFR_IO_ADDR(SREG)));
	asm volatile("pop r16");
	reti();

_USART_INT_RECEIVED_FRAME_ID:
	// We are reading a frame ID and should handle the 0xFF and 0 cases
	asm volatile("tst r16");
	asm volatile goto("breq %l0" ::: "cc" : _USART_INT_FRAME_ID_ZERO);
	asm volatile("inc r16");
	asm volatile goto("breq %l0" ::: "cc" : _USART_INT_RECEIVED_SYNC_START);

	// The frame is normal and ID is decremented
	asm volatile("subi r16, %0" :: "M" (2));

	// Set saving to zero and continue like a normal byte
	asm volatile("std Z+%0, r17" :: "I" (SAVING_OFFSET));
	asm volatile goto("rjmp %l0" ::: "cc" : _USART_INT_RECEIVED_REGULAR_BYTE);

_USART_INT_FRAME_ID_ZERO:
	// We need to set saving to nonzero and continue like normal
	asm volatile("dec r16");
	asm volatile("std Z+%0, r16" :: "I" (SAVING_OFFSET));
	asm volatile("inc r16");
	asm volatile goto("rjmp %l0" ::: "cc" : _USART_INT_RECEIVED_REGULAR_BYTE);

_USART_INT_RECEIVED_SYNC_START:
	// We need to set syncing to nonzero and send the received data without doing anything else
	asm volatile("dec r16");
	asm volatile("std Z+%0, r16" :: "I" (SYNCING_OFFSET));
	asm volatile goto("rjmp %l0" ::: "cc" : _USART_INT_SEND_CHAR);

_USART_INT_SYNCING_IS_TRUE:
	// If recieved byte is not magic, just send it
	asm volatile("mov r17, r16");
	asm volatile("cpi r17, %0" :: "M" (SYNC_END_CHARACTER));
	asm volatile goto("brne %l0" ::: "cc" : _USART_INT_SEND_CHAR);

	// Otherwise we should reset the state-machine and send it
	asm volatile("clr r17");
	asm volatile("std Z+%0, r17" :: "I" (SYNCING_OFFSET));
	asm volatile("std Z+%0, r17" :: "I" (SAVING_OFFSET));
	asm volatile("std Z+%0, r17" :: "I" (RECV_IDX_OFFSET));
	asm volatile goto("rjmp %l0" ::: "cc" : _USART_INT_SEND_CHAR);

_USART_INT_SET_BUFFER_READY:
	asm volatile("clr r17");
	asm volatile("std Z+%0, r17" :: "I" (SAVING_OFFSET));
	asm volatile("std Z+%0, r17" :: "I" (RECV_IDX_OFFSET));
	asm volatile("dec r17");
	asm volatile("std Z+%0, r17" :: "I" (BUFFER_READY_OFFSET));
	asm volatile goto("rjmp %l0" ::: "cc" : _USART_INT_BUFFER_READY_DONE);
}

ISR(TCA0_CMP0_vect, ISR_NAKED)
{
	static volatile uint8_t *portvals_ptr = portvals;
	static uint8_t pwm_column = 8;

	// Clear interrupt flag with minimum latency
	asm volatile("push ZL");
	asm volatile("ldi r30, %0" :: "M" (TCA_SINGLE_CMP0_bm));
	asm volatile("sts %0, r30" :: "m" (TCA0_SINGLE_INTFLAGS));

	// Save utilized registers (including SREG)
	asm volatile("push r0");
	asm volatile("in r0, %0" :: "I" (_SFR_IO_ADDR(SREG)));
	asm volatile("push r0");
	asm volatile("push r1");
	asm volatile("push r16");
	asm volatile("push ZH");

	// Set Z to portvals_ptr
	asm volatile("lds ZL, %0" :: "m" (portvals_ptr));
	asm volatile("lds ZH, %0+1" :: "m" (portvals_ptr));

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

	// Set Z to the correct portvals tail
	asm volatile("lds ZL, %0" :: "m" (portvals_tail_ptr));
	asm volatile("lds ZH, %0+1" :: "m" (portvals_tail_ptr));

	// And subtract our product from it
	asm volatile("sub ZL, r0");
	asm volatile("sbc ZH, r1");
	
_TCA_INT_END:
	// Save portvals_ptr
	asm volatile("sts %0, r30" :: "m" (portvals_ptr));
	asm volatile("sts %0+1, r31" :: "m" (portvals_ptr));

	// Restore the utilized registers
	asm volatile("pop ZH");
	asm volatile("pop r16");
	asm volatile("pop r1");
	asm volatile("pop r0");
	asm volatile("out %0, r0" :: "I" (_SFR_IO_ADDR(SREG)));
	asm volatile("pop r0");
	asm volatile("pop ZL");
	reti();
}

static inline void parse_buffer(void)
{
	// Don't do anything if the buffer is not ready to parse
	if (recv_buffer[BUFFER_READY_OFFSET] == 0) return;

	// Something is wrong if the first value is nonzero
	if (recv_buffer[0] == 0){
		volatile uint8_t *temp_portvals;
		if (portvals_tail_ptr == &portvals[4*10*8]) temp_portvals = &portvals[4*10*8];
		else temp_portvals = &portvals[0];

		// Now iterate through the data buffer
		for (uint8_t parsecolumn = 0; parsecolumn < 8; parsecolumn++)
		{
			// Start by decompressing the double-word RGBY quadruplet into RGB triplet
			uint8_t currentcol[9];
			for (uint8_t colpixel = 0; colpixel < 3; colpixel++)
			{
				uint8_t pixelh = recv_buffer[1 + 2*(colpixel+3*parsecolumn)];
				uint8_t pixell = recv_buffer[2 + 2*(colpixel+3*parsecolumn)];
				uint8_t yyyy = ((pixell & 0x0F)+1);
				currentcol[colpixel*3 + 0] = (pixelh >> 4) * yyyy; // Red
				currentcol[colpixel*3 + 1] = (pixelh & 0x0F) * yyyy; // Green
				currentcol[colpixel*3 + 2] = (pixell >> 4) * yyyy; // Blue
			}
			
			uint8_t compareval = 0;
			for (uint8_t parsestep = 0; parsestep < 10; parsestep++)
			{
				uint8_t tempval_porta = PORTA_OUTPUT_m & colmasksa[parsecolumn];
				uint8_t tempval_portd = PORTD_OUTPUT_m & colmasksd[parsecolumn];
				uint8_t tempval_portf = PORTF_OUTPUT_m & colmasksf[parsecolumn];
				uint8_t minval = 240;
				for (uint8_t row = 0; row < 9; row++)
				{
					if (currentcol[row] > compareval)
					{
						switch (row)
						{
							case 0:
							tempval_porta &= ~R1_PINA_m;
							break;
							case 1:
							tempval_portd &= ~G1_PIND_m;
							break;
							case 2:
							tempval_porta &= ~B1_PINA_m;
							break;
							case 3:
							tempval_portd &= ~R2_PIND_m;
							break;
							case 4:
							tempval_portd &= ~G2_PIND_m;
							break;
							case 5:
							tempval_porta &= ~B2_PINA_m;
							break;
							case 6:
							tempval_portd &= ~R3_PIND_m;
							break;
							case 7:
							tempval_portd &= ~G3_PIND_m;
							break;
							case 8:
							tempval_porta &= ~B3_PINA_m;
							break;
						}
						if (currentcol[row] < minval)
						{
							minval = currentcol[row];
						}
					}
				}

				// Write out the port values
				temp_portvals[40*parsecolumn + 4*parsestep + 0] = tempval_porta;
				temp_portvals[40*parsecolumn + 4*parsestep + 1] = tempval_portd;
				temp_portvals[40*parsecolumn + 4*parsestep + 2] = tempval_portf;
				
				// Check if we have filled in the last part
				if (minval == 240)
				{
					// Set the compare to loop back around to 0 and terminate the loop
					temp_portvals[40*parsecolumn + 4*parsestep + 3] = 0;
					break;
				}
				
				// Otherwise set the compare value and continue
				temp_portvals[40*parsecolumn + 4*parsestep + 3] = minval;
				compareval = minval;
			}
		}
		
		if (portvals_tail_ptr == &portvals[4*10*8]) portvals_tail_ptr = &portvals[4*10*8*2];
		else portvals_tail_ptr = &portvals[4*10*8];
	}

	// We are free to indicate that we have finished servicing the buffer
	recv_buffer[BUFFER_READY_OFFSET] = 0;
	return;
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

	CPUINT.LVL1VEC = TCA0_CMP0_vect_num;

	// USART is 8N1 asynchronous for testing
	// Should be 8N1 synchronous for final
	uint32_t baud = 87; // 87 = round(64 * 20 MHz / (16 * 921600 Hz))
	USART1.BAUD = (uint16_t)((baud * (1024 + SIGROW.OSC20ERR5V))/1024);
	USART1.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_SBMODE_1BIT_gc | USART_CHSIZE_8BIT_gc;
	USART1.CTRLB = USART_TXEN_bm | USART_RXEN_bm | USART_RXMODE_NORMAL_gc;
	USART1.CTRLA = USART_RXCIE_bm;

	// 60 Hz refresh rate, 20 MHz clock, and 240 dimming steps per period
	TCA0.SINGLE.PER = 41519; // 240 * floor(20 MHz / (8 * 60 Hz * 240)) - 1
	TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP0_bm;
	TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;

	recv_buffer[SYNCING_OFFSET] = 1;

	sei();

	while (1) parse_buffer();
}
