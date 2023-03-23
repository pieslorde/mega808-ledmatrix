#include "main.h"

#define SYNC_END_CHARACTER (0xAA)
#define FRAME_LENGTH (1+2*24+1) // Frame ID, 24 LEDs with 2-byte values, and CRC-8

#define COLUMNS 8
#define ROWS 9
#define MAX_DIM_STEPS (ROWS+1)
#define BYTES_PER_DIM_STEP 4
#define BYTES_PER_PORTVALS_BUFFER (BYTES_PER_DIM_STEP*MAX_DIM_STEPS*COLUMNS)
#define PORTVALS_LENGTH (BYTES_PER_PORTVALS_BUFFER*2)

// Buffer of USART data terminated with buffer_ready, saving, recv_idx, and syncing
volatile uint8_t recv_buffer[FRAME_LENGTH] __attribute__((aligned (64)));

// Used to indicate to the main context that the buffer contains data ready for processing
register uint8_t buffer_ready asm("r11");

// 10 quadruplets of PORTA value, PORTD value, PORTF value, and PWM compare value
// 8 dectuplets for each column
// Two buffers terminated by an offset from the tail
volatile uint8_t portvals[PORTVALS_LENGTH];
volatile uint8_t *portvals_tail_ptr = &portvals[4*10*8];

// Lookup tables for column masks
static const uint8_t colmasksa[8] = {~COL1_PINA_m, ~COL2_PINA_m, ~0, ~0, ~0, ~0, ~COL7_PINA_m, ~0};
static const uint8_t colmasksd[8] = {~0, ~0, ~0, ~0, ~COL5_PIND_m, (uint8_t)(~COL6_PIND_m), ~0, ~COL8_PIND_m};
static const uint8_t colmasksf[8] = {~0, ~0, ~COL3_PINF_m, ~COL4_PINF_m, ~0, ~0, ~0, ~0};

// CRC-8 lookup table - Used to validate recieved frame before we process it
static const uint8_t crc8[256] = {
	0x00, 0xCF, 0x51, 0x9E, 0xA2, 0x6D, 0xF3, 0x3C, 0x8B, 0x44, 0xDA, 0x15, 0x29, 0xE6, 0x78, 0xB7,
	0xD9, 0x16, 0x88, 0x47, 0x7B, 0xB4, 0x2A, 0xE5, 0x52, 0x9D, 0x03, 0xCC, 0xF0, 0x3F, 0xA1, 0x6E,
	0x7D, 0xB2, 0x2C, 0xE3, 0xDF, 0x10, 0x8E, 0x41, 0xF6, 0x39, 0xA7, 0x68, 0x54, 0x9B, 0x05, 0xCA,
	0xA4, 0x6B, 0xF5, 0x3A, 0x06, 0xC9, 0x57, 0x98, 0x2F, 0xE0, 0x7E, 0xB1, 0x8D, 0x42, 0xDC, 0x13,
	0xFA, 0x35, 0xAB, 0x64, 0x58, 0x97, 0x09, 0xC6, 0x71, 0xBE, 0x20, 0xEF, 0xD3, 0x1C, 0x82, 0x4D,
	0x23, 0xEC, 0x72, 0xBD, 0x81, 0x4E, 0xD0, 0x1F, 0xA8, 0x67, 0xF9, 0x36, 0x0A, 0xC5, 0x5B, 0x94,
	0x87, 0x48, 0xD6, 0x19, 0x25, 0xEA, 0x74, 0xBB, 0x0C, 0xC3, 0x5D, 0x92, 0xAE, 0x61, 0xFF, 0x30,
	0x5E, 0x91, 0x0F, 0xC0, 0xFC, 0x33, 0xAD, 0x62, 0xD5, 0x1A, 0x84, 0x4B, 0x77, 0xB8, 0x26, 0xE9,
	0x3B, 0xF4, 0x6A, 0xA5, 0x99, 0x56, 0xC8, 0x07, 0xB0, 0x7F, 0xE1, 0x2E, 0x12, 0xDD, 0x43, 0x8C,
	0xE2, 0x2D, 0xB3, 0x7C, 0x40, 0x8F, 0x11, 0xDE, 0x69, 0xA6, 0x38, 0xF7, 0xCB, 0x04, 0x9A, 0x55,
	0x46, 0x89, 0x17, 0xD8, 0xE4, 0x2B, 0xB5, 0x7A, 0xCD, 0x02, 0x9C, 0x53, 0x6F, 0xA0, 0x3E, 0xF1,
	0x9F, 0x50, 0xCE, 0x01, 0x3D, 0xF2, 0x6C, 0xA3, 0x14, 0xDB, 0x45, 0x8A, 0xB6, 0x79, 0xE7, 0x28,
	0xC1, 0x0E, 0x90, 0x5F, 0x63, 0xAC, 0x32, 0xFD, 0x4A, 0x85, 0x1B, 0xD4, 0xE8, 0x27, 0xB9, 0x76,
	0x18, 0xD7, 0x49, 0x86, 0xBA, 0x75, 0xEB, 0x24, 0x93, 0x5C, 0xC2, 0x0D, 0x31, 0xFE, 0x60, 0xAF,
	0xBC, 0x73, 0xED, 0x22, 0x1E, 0xD1, 0x4F, 0x80, 0x37, 0xF8, 0x66, 0xA9, 0x95, 0x5A, 0xC4, 0x0B,
	0x65, 0xAA, 0x34, 0xFB, 0xC7, 0x08, 0x96, 0x59, 0xEE, 0x21, 0xBF, 0x70, 0x4C, 0x83, 0x1D, 0xD2
};

// Reserved globally so we don't have to push/pop it
register uint8_t USART_RECEIVED_DATA asm("r2");

// Reserved globally so it can be used persistently
// Zero means we are not syncing or saving, (most important path)
// Positive means we are saving, and (second most important path)
// Negative value means we are syncing (least important path)
register int8_t USART_STATE asm("r9");

// Reserved globally so it can be used persistently
register uint8_t USART_INDEX asm("r10");

// GPIOR0 and GPIOR1 are used to save Z pointer in TCA0 ISR
// GPIOR2 and GPIOR3 are used to save r0:r1 MUL accumulator pair in TCA0 ISR

// Must be a word-sized pair so we can copy it to a pointer pair in one cycle
register uint8_t TCA_PTRL asm("r4");
register uint8_t TCA_PTRH asm("r5");
register uint8_t TCA_COLUMN asm("r6");
// Saving precious time in TCA interrupt by holding constants in registers
register uint8_t TCA_40 asm("r7");
register uint8_t TCA_173 asm("r8");

FUSES = {
	.WDTCFG = FUSE_WDTCFG_DEFAULT,
	.BODCFG = LVL_BODLEVEL7_gc | SAMPFREQ_1KHZ_gc | ACTIVE_ENABLED_gc | SLEEP_ENABLED_gc,
	.OSCCFG = FREQSEL_20MHZ_gc,
	.SYSCFG0 = CRCSRC_NOCRC_gc | RSTPINCFG_RST_gc,
	.SYSCFG1 = SUT_64MS_gc,
	.APPEND = FUSE_APPEND_DEFAULT,
	.BOOTEND = FUSE_BOOTEND_DEFAULT,
};

ISR(USART1_RXC_vect, ISR_NAKED)
{
	register uint8_t PTR_L asm("r28");
	register uint8_t PTR_H asm("r29");
	register uint8_t TEMP_VAR asm("r17");

	// Save SREG using USART_RECEIVED_DATA
	asm volatile("in %0, %1" : "=r" (USART_RECEIVED_DATA) : "I" (_SFR_IO_ADDR(SREG)));
	asm volatile("push %0" :: "r" (USART_RECEIVED_DATA));

	// Check for frame error
	asm volatile("lds %0, %1" : "=r" (USART_RECEIVED_DATA) : "m" (USART1_RXDATAH));
	asm volatile("sbrc %0, %1" :: "r" (USART_RECEIVED_DATA), "M" (USART_FERR_bp));
	asm volatile goto("rjmp %l0" :::: _USART_INT_FRAME_ERROR);

	// Read the new data into USART_RECEIVED_DATA
	asm volatile("lds %0, %1" : "=r" (USART_RECEIVED_DATA) : "m" (USART1_RXDATAL));

	// Save the rest
	asm volatile("push %0" :: "r" (TEMP_VAR));
	asm volatile("push %0" :: "r" (PTR_L));
	asm volatile("push %0" :: "r" (PTR_H));

	// Load base of recv_buffer to pointer register
	asm volatile("ldi %0, lo8(%1)" : "=d" (PTR_L) : "m" (recv_buffer));
	asm volatile("ldi %0, hi8(%1)" : "=d" (PTR_H) : "m" (recv_buffer));

	// Test the index register and if it is zero we have to handle the frame ID
	asm volatile("tst %0" :: "r" (USART_INDEX) : "cc");
	asm volatile goto("breq %l0" :::: _USART_INT_RECEIVED_FRAME_ID);

	// Test the state machine register and jump to the relevant subroutines if USART_STATE is non-zero
	asm volatile("tst %0" :: "r" (USART_STATE) : "cc");
	asm volatile goto("brmi %l0" :::: _USART_INT_SYNCING_IS_TRUE);
	asm volatile goto("brne %l0" :::: _USART_INT_SAVE_DATA);

_USART_INT_INCREMENT_INDEX:
	// Frame IDs and saving data have both been addressed by this point
	// We can increment the index and load the index limit to TEMP_VAR
	asm volatile("inc %0" : "+r" (USART_INDEX) :: "cc");
	asm volatile("ldi %0, %1" : "=r" (TEMP_VAR) : "M" (FRAME_LENGTH-1));
	asm volatile("cp %0, %1" : "+r" (TEMP_VAR) : "r" (USART_INDEX) : "cc");
	asm volatile goto("brlt %l0" :::: _USART_INT_RESET_INDEX_AND_SEND);

_USART_INT_SEND_CHAR:
	// While DREIF is 0, busy wait. This might not be required when operating synchronously
	asm volatile("lds %0, %1" : "=r" (TEMP_VAR) : "m" (USART1_STATUS));
	asm volatile("sbrs %0, %1" :: "r" (TEMP_VAR), "M" (USART_DREIF_bp)); // Continue if bit is set
	asm volatile goto("rjmp %l0" :::: _USART_INT_SEND_CHAR);

	// Write USART_RECEIVED_DATA to TXDATAL
	asm volatile("sts %0, %1" :: "m" (USART1_TXDATAL), "r" (USART_RECEIVED_DATA));

_USART_INT_RETURN:
	// Restore registers that were saved
	asm volatile("pop %0" : "=r" (PTR_H));
	asm volatile("pop %0" : "=r" (PTR_L));
	asm volatile("pop %0" : "=r" (TEMP_VAR));

	// Restore SREG and leave
	asm volatile("pop %0" :: "r" (USART_RECEIVED_DATA));
	asm volatile("out %0, %1" :: "I" (_SFR_IO_ADDR(SREG)), "r" (USART_RECEIVED_DATA) : "cc");
	reti();

	// --- Subroutines for less important paths --- //

_USART_INT_SAVE_DATA:
	// Add the index to the pointer address and store the received data
	asm volatile("add %0, %1" : "+r" (PTR_L) : "r" (USART_INDEX) : "cc");
	asm volatile("st %a0, %1" :: "y" (PTR_L), "r" (USART_RECEIVED_DATA));
	
	// Increment the index here because we don't want to forward frames addressed to us
	asm volatile("inc %0" : "+r" (USART_INDEX) :: "cc");
	asm volatile("ldi %0, %1" : "=r" (TEMP_VAR) : "M" (FRAME_LENGTH-1));
	asm volatile("cp %0, %1" : "+r" (TEMP_VAR) : "r" (USART_INDEX) : "cc");
	asm volatile goto("brlt %l0" :::: _USART_INT_RESET_INDEX);

	// And leave
	asm volatile goto("rjmp %l0" :::: _USART_INT_RETURN);

_USART_INT_RESET_INDEX:
	// Test the state again if we are saving
	asm volatile("tst %0" :: "r" (USART_STATE) : "cc");
	asm volatile goto("breq %l0" :::: _USART_INT_SKIP_BUFFER_READY);

	// We were saving, so we need to set the buffer ready flag and clear the state
	asm volatile("clr %0" : "=r" (USART_STATE) :: "cc");
	asm volatile("clr %0" : "=r" (buffer_ready) :: "cc");
	asm volatile("inc %0" : "+r" (buffer_ready) :: "cc");

_USART_INT_SKIP_BUFFER_READY:
	// Reset index and return
	asm volatile("clr %0" : "=r" (USART_INDEX) :: "cc");
	asm volatile goto("rjmp %l0" :::: _USART_INT_RETURN);

_USART_INT_RESET_INDEX_AND_SEND:
	// Test the state again if we are saving
	asm volatile("tst %0" :: "r" (USART_STATE) : "cc");
	asm volatile goto("breq %l0" :::: _USART_INT_SKIP_BUFFER_READY_AND_SEND);

	// We were saving, so we need to set the buffer ready flag and clear the state
	asm volatile("clr %0" : "=r" (USART_STATE) :: "cc");
	asm volatile("clr %0" : "=r" (buffer_ready) :: "cc");
	asm volatile("inc %0" : "+r" (buffer_ready) :: "cc");

_USART_INT_SKIP_BUFFER_READY_AND_SEND:
	// Reset index and return
	asm volatile("clr %0" : "=r" (USART_INDEX) :: "cc");
	asm volatile goto("rjmp %l0" :::: _USART_INT_SEND_CHAR);

_USART_INT_RECEIVED_FRAME_ID:
	// We are reading a frame ID and should handle the 0xFF and 0 cases
	asm volatile("tst %0" :: "r" (USART_RECEIVED_DATA) : "cc");
	asm volatile goto("breq %l0" :::: _USART_INT_FRAME_ID_ZERO);
	asm volatile("inc %0" : "+r" (USART_RECEIVED_DATA) :: "cc");
	asm volatile goto("breq %l0" :::: _USART_INT_RECEIVED_SYNC_START);

	// The frame is normal and ID is decremented
	asm volatile("dec %0" : "+r" (USART_RECEIVED_DATA) :: "cc");
	asm volatile("dec %0" : "+r" (USART_RECEIVED_DATA) :: "cc");

	// Set state to not saving or syncing and we can skip the state checker
	asm volatile("clr %0" :: "r" (USART_STATE) : "cc");
	asm volatile goto("rjmp %l0" :::: _USART_INT_INCREMENT_INDEX);

_USART_INT_FRAME_ID_ZERO:
	// Set state to 1 indicating we are saving
	asm volatile("inc %0" : "+r" (USART_RECEIVED_DATA) :: "cc");
	asm volatile("mov %0, %1" : "=r" (USART_STATE) : "r" (USART_RECEIVED_DATA));
	asm volatile("dec %0" : "+r" (USART_RECEIVED_DATA) :: "cc");
	
	// Jump to data saving routine
	asm volatile goto("rjmp %l0" :::: _USART_INT_SAVE_DATA);

_USART_INT_RECEIVED_SYNC_START:
	// Set the state to syncing and set index to 255 so we don't handle every byte as a frame ID
	asm volatile("dec %0" : "+r" (USART_RECEIVED_DATA) :: "cc");
	asm volatile("mov %0, %1" : "=r" (USART_STATE) : "r" (USART_RECEIVED_DATA));
	asm volatile("inc %0" : "+r" (USART_INDEX));
	asm volatile goto("rjmp %l0" :::: _USART_INT_SEND_CHAR);

_USART_INT_SYNCING_IS_TRUE:
	// If recieved byte is not magic, just send it
	asm volatile("mov %0, %1" : "=r" (TEMP_VAR) : "r" (USART_RECEIVED_DATA));
	asm volatile("cpi %0, %1" : "+r" (TEMP_VAR) : "M" (SYNC_END_CHARACTER) : "cc");
	asm volatile goto("brne %l0" :::: _USART_INT_SEND_CHAR);

	// Otherwise we should reset the state-machine and send it
	asm volatile("clr %0" : "=r" (USART_STATE) :: "cc");
	asm volatile("clr %0" : "=r" (USART_INDEX) :: "cc");
	asm volatile goto("rjmp %l0" :::: _USART_INT_SEND_CHAR);

_USART_INT_FRAME_ERROR:
	// Clear receive complete flag
	asm volatile("lds %0, %1" : "=r" (USART_RECEIVED_DATA) : "m" (USART1_RXDATAL));
	// Restore SREG and leave
	asm volatile("pop %0" :: "r" (USART_RECEIVED_DATA));
	asm volatile("out %0, %1" :: "I" (_SFR_IO_ADDR(SREG)), "r" (USART_RECEIVED_DATA) : "cc");
	reti();
}

ISR(TCA0_CMP0_vect, ISR_NAKED)
{
	register uint8_t ACCUM_L asm("r0");
	register uint8_t ACCUM_H asm("r1");
	register uint8_t PTR_L asm("r30");
	register uint8_t PTR_H asm("r31");

	// Save PTR_L to GPIOR0
	asm volatile("out %0, %1" :: "I" (_SFR_IO_ADDR(GPIOR0)), "r" (PTR_L));

	// Clear interrupt flag with minimum latency
	asm volatile("ldi %0, %1" : "=d" (PTR_L) : "M" (TCA_SINGLE_CMP0_bm));
	asm volatile("sts %0, %1" :: "m" (TCA0_SINGLE_INTFLAGS), "r" (PTR_L));

	// Save utilized registers to remaining GPIORs and save SREG to stack
	asm volatile("out %0, %1" :: "I" (_SFR_IO_ADDR(GPIOR1)), "r" (PTR_H));
	asm volatile("out %0, %1" :: "I" (_SFR_IO_ADDR(GPIOR2)), "r" (ACCUM_L));
	asm volatile("out %0, %1" :: "I" (_SFR_IO_ADDR(GPIOR3)), "r" (ACCUM_H));
	asm volatile("in %0, %1" : "=r" (ACCUM_L) : "I" (_SFR_IO_ADDR(SREG)));
	asm volatile("push %0" :: "r" (ACCUM_L));

	// Set Z to portvals_ptr
	asm volatile("movw %0, %2" : "=r" (PTR_L), "=r" (PTR_H) : "r" (TCA_PTRL), "r" (TCA_PTRH));

	// Write ports out
	asm volatile("ld %0, Z+" : "=r" (ACCUM_L));
	asm volatile("out %0, %1" :: "I" (_SFR_IO_ADDR(VPORTA_OUT)), "r" (ACCUM_L));
	asm volatile("ld %0, Z+" : "=r" (ACCUM_L));
	asm volatile("out %0, %1" :: "I" (_SFR_IO_ADDR(VPORTD_OUT)), "r" (ACCUM_L));
	asm volatile("ld %0, Z+" : "=r" (ACCUM_L));
	asm volatile("out %0, %1" :: "I" (_SFR_IO_ADDR(VPORTF_OUT)), "r" (ACCUM_L));

	// Get the next compare value and multiply by 173
	asm volatile("ld %0, Z+" : "=r" (ACCUM_L));
	// asm volatile("ldi r16, %0" :: "M" (173));
	asm volatile("mul %0, %2" : "+r" (ACCUM_L), "=r" (ACCUM_H) : "r" (TCA_173));

	// Write the compare value to TCA0.SINGLE.CMP0
	asm volatile("sts %0, %1" :: "m" (TCA0_SINGLE_CMP0L), "r" (ACCUM_L));
	asm volatile("sts %0, %1" :: "m" (TCA0_SINGLE_CMP0H), "r" (ACCUM_H));

	// If compare value is nonzero, branch to the end
	asm volatile goto("breq %l0" ::: "cc" : _TCA_INT_COMPARE_ZERO);
	
_TCA_INT_END:
	// Save portvals_ptr
	asm volatile("movw %0, %2" : "=r" (TCA_PTRL), "=r" (TCA_PTRH) : "r" (PTR_L), "r" (PTR_H));

	// Pop SREG from stack and restore it
	asm volatile("pop %0" : "=r" (ACCUM_L));
	asm volatile("out %1, %0" : "=r" (ACCUM_L) : "I" (_SFR_IO_ADDR(SREG)));

	// Restore utilized regs from GPIORs
	asm volatile("in %0, %1" : "=r" (ACCUM_H) : "I" (_SFR_IO_ADDR(GPIOR3)));
	asm volatile("in %0, %1" : "=r" (ACCUM_L) : "I" (_SFR_IO_ADDR(GPIOR2)));
	asm volatile("in %0, %1" : "=r" (PTR_H) : "I" (_SFR_IO_ADDR(GPIOR1)));
	asm volatile("in %0, %1" : "=r" (PTR_L) : "I" (_SFR_IO_ADDR(GPIOR0)));
	reti();

_TCA_INT_COMPARE_ZERO:
	// Grab the column counter and decrement
	// asm volatile("lds r16, %0" :: "m" (pwm_column));
	// asm volatile("dec r16");
	asm volatile("dec %0" : "+r" (TCA_COLUMN));

	// If have not overflowed, skip ahead
	asm volatile goto("brne %l0" ::: "cc" : _TCA_INT_NO_COLUMN_OVERFLOW);

	// Otherwise we must load 8 to the column register
	register uint8_t TEMP_LOAD_REG asm("r16");
	asm volatile("push %0" :: "r" (TEMP_LOAD_REG));
	asm volatile("ldi %0, %1" : "=d" (TEMP_LOAD_REG) : "M" (8));
	asm volatile("mov %0, %1" : "=r" (TCA_COLUMN) : "r" (TEMP_LOAD_REG));
	asm volatile("pop %0" : "=r" (TEMP_LOAD_REG));

_TCA_INT_NO_COLUMN_OVERFLOW:
	// Multiply pwm_column by 40
	asm volatile("mul %2, %3" : "=r" (ACCUM_L), "=r" (ACCUM_H) : "r" (TCA_COLUMN), "r" (TCA_40));

	// Set Z to the correct portvals tail
	asm volatile("lds %0, %1" : "=r" (PTR_L) : "m" (portvals_tail_ptr));
	asm volatile("lds %0, %1+1" : "=r" (PTR_H) : "m" (portvals_tail_ptr));

	// And subtract our product from it
	asm volatile("sub %0, %1" : "+r" (PTR_L) : "r" (ACCUM_L));
	asm volatile("sbc %0, %1" : "+r" (PTR_H) : "r" (ACCUM_H));
	
	// And end normally
	asm volatile goto("rjmp %l0" ::: "cc" : _TCA_INT_END);
}

void init_clock(void)
{
	CCP = CCP_IOREG_gc;
	// CLKCTRL.MCLKCTRLB = CLKCTRL_PDIV_64X_gc | CLKCTRL_PEN_bm; // Slow the CPU to 1/64 speed for measuring CLKOUT
	CLKCTRL.MCLKCTRLB = 0; // Let the CPU run at full tilt
}

void init_gpio_outputs(void)
{
	// Outputs are active low, so we set them before enabling
	VPORTA.OUT = PORTA_OUTPUT_m;
	VPORTA.DIR = PORTA_OUTPUT_m;
	VPORTD.OUT = PORTD_OUTPUT_m;
	VPORTD.DIR = PORTD_OUTPUT_m;
	VPORTF.OUT = PORTF_OUTPUT_m;
	VPORTF.DIR = PORTF_OUTPUT_m;

	VPORTC.DIR = PORTC_OUTPUT_m;
}

void preload_portvals(void)
{
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
}

void wait_for_buffer_ready(void)
{
_BUFFER_READY_LOOP:
	asm volatile goto(
		"tst %0\n\t"
		"breq %l1"
		:
		: "r" (buffer_ready)
		: "cc"
		: _BUFFER_READY_LOOP);
}

uint8_t check_crc(void)
{
	uint8_t crc = 105;
	for (uint8_t receive_index = 1; receive_index < FRAME_LENGTH; receive_index++){
		crc = crc8[crc ^ recv_buffer[receive_index]];
	}
	return crc;
}

void init_usart(void)
{
	// USART is 8N1 asynchronous for testing
	// Should be 8N1 synchronous for final

#define USART_BAUD_RATE(BAUD_RATE) ((float)(20000000UL * 64UL / (16UL * (float)BAUD_RATE)) + 0.5)

	uint32_t baud = USART_BAUD_RATE(115200);
	int8_t frequency_correction = SIGROW.OSC20ERR5V;
	baud *= (1024 + frequency_correction);
	baud /= 1024; // The extra 1024 in the macro needs to be divided back out
	USART1.BAUD = (uint16_t)baud;
	USART1.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_SBMODE_1BIT_gc | USART_CHSIZE_8BIT_gc;
	USART1.CTRLB = USART_TXEN_bm | USART_RXEN_bm | USART_RXMODE_NORMAL_gc;
	USART1.CTRLA = USART_RXCIE_bm;

	USART_STATE = -1; // We should boot up in syncing mode
}

void init_timer(void)
{
	CPUINT.LVL1VEC = TCA0_CMP0_vect_num;

	// 60 Hz refresh rate, 20 MHz clock, and 240 dimming steps per period
	TCA0.SINGLE.PER = 41519; // 240 * floor(20 MHz / (8 * 60 Hz * 240)) - 1
	TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP0_bm;
	TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;

	TCA_PTRL = (uint8_t)(((uint16_t)portvals));
	TCA_PTRH = (uint8_t)(((uint16_t)portvals>>8));
	TCA_COLUMN = 8;
	TCA_40 = 40;
	TCA_173 = 173;
}

int main(void)
{
	init_clock();
	init_gpio_outputs();

#ifdef _TEST_CLKOUT_
	CCP = CCP_IOREG_gc;
	CLKCTRL.MCLKCTRLA = CLKCTRL_CLKOUT_bm | CLKCTRL_CLKSEL_OSC20M_gc;
	while (1);
#endif

#ifdef __IOTEST_H__
	// This test function does not return
	dim_test();
#endif

	preload_portvals();
	init_usart();
	init_timer();

	while (!(USART1.STATUS & USART_DREIF_bm));
	USART1.TXDATAL = TCA_40;

	sei();

	while (1){
		// Don't do anything if the buffer is not ready to parse
		wait_for_buffer_ready();

		// Check the CRC. Result should be zero
		if(check_crc() != 0){
			asm volatile("clr %0" : "=r" (buffer_ready) :: "cc");
			continue;
		}

		// Something is wrong if the first value is nonzero
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
		
		// Swap the buffer because the one we just filled is complete
		if (portvals_tail_ptr == &portvals[4*10*8]) portvals_tail_ptr = &portvals[4*10*8*2];
		else portvals_tail_ptr = &portvals[4*10*8];
		
		// We are free to indicate that we have finished servicing the buffer
		asm volatile("clr %0" : "=r" (buffer_ready) :: "cc");
	}
}
