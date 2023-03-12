#ifndef _PINDEFS_H_
#define _PINDEFS_H_

// PORTA definitions
#define R1_PINA_m (1 << 7)
#define COL7_PINA_m (1 << 6)
#define B1_PINA_m (1 << 4)
#define B2_PINA_m (1 << 3)
#define B3_PINA_m (1 << 2)
#define COL2_PINA_m (1 << 1)
#define COL1_PINA_m (1 << 0)

#define B_PORTA_m (B1_PINA_m | B2_PINA_m | B3_PINA_m)
#define R_PORTA_m (R1_PINA_m)
#define COL_PORTA_m (COL7_PINA_m | COL2_PINA_m | COL1_PINA_m)
#define ROW_PORTA_m (B_PORTA_m | R_PORTA_m)

#define PORTA_OUTPUT_m (ROW_PORTA_m | COL_PORTA_m)

// PORTC definitions
#define USART1_TX_PINC_m (1 << 0)
#define USART1_RX_PINC_m (1 << 1)
#define USART1_CLK_PINC_m (1 << 2)

#define PORTC_OUTPUT_m (USART1_TX_PINC_m)

// PORTD definitions
#define COL6_PIND_m (1 << 7)
#define COL5_PIND_m (1 << 6)
#define COL8_PIND_m (1 << 5)
#define G3_PIND_m (1 << 4)
#define G2_PIND_m (1 << 3)
#define G1_PIND_m (1 << 2)
#define R3_PIND_m (1 << 1)
#define R2_PIND_m (1 << 0)

#define R_PORTD_m (R3_PIND_m | R2_PIND_m)
#define G_PORTD_m (G3_PIND_m | G2_PIND_m | G1_PIND_m)
#define COL_PORTD_m (COL6_PIND_m | COL5_PIND_m | COL8_PIND_m)
#define ROW_PORTD_m (R_PORTD_m | G_PORTD_m)

#define PORTD_OUTPUT_m (ROW_PORTD_m | COL_PORTD_m)

// PORTF definitions
#define COL4_PINF_m (1 << 1)
#define COL3_PINF_m (1 << 0)

#define COL_PORTF_m (COL4_PINF_m | COL3_PINF_m)

#define PORTF_OUTPUT_m (COL_PORTF_m)

#endif //_PINDEFS_H_
