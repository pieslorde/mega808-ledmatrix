#ifndef _PINDEFS_H_
#define _PINDEFS_H_

// PORTA definitions
#define G1_PIN_m (1 << 7)
#define COL7_PIN_m (1 << 6)
#define R1_PIN_m (1 << 4)
#define R2_PIN_m (1 << 3)
#define R3_PIN_m (1 << 2)
#define COL2_PIN_m (1 << 1)
#define COL1_PIN_m (1 << 0)

#define R_PORTA_m (R1_PIN_m | R2_PIN_m | R3_PIN_m)
#define COL_PORTA_m (COL7_PIN_m | COL2_PIN_m | COL1_PIN_m)
#define G_PORTA_m (G1_PIN_m)

#define PORTA_OUTPUT_m (R_PORTA_m | COL_PORTA_m | G_PORTA_m)

// PORTD definitions
#define COL6_PIN_m (1 << 7)
#define COL5_PIN_m (1 << 6)
#define COL8_PIN_m (1 << 5)
#define B3_PIN_m (1 << 4)
#define B2_PIN_m (1 << 3)
#define B1_PIN_m (1 << 2)
#define G3_PIN_m (1 << 1)
#define G2_PIN_m (1 << 0)

#define COL_PORTD_m (COL6_PIN_m | COL5_PIN_m | COL8_PIN_m)
#define B_PORTD_m (B3_PIN_m | B2_PIN_m | B1_PIN_m)
#define G_PORTD_m (G3_PIN_m | G2_PIN_m)

#define PORTD_OUTPUT_m (COL_PORTD_m | B_PORTD_m | G_PORTD_m)

// PORTF definitions
#define COL4_PIN_m (1 << 4)
#define COL3_PIN_m (1 << 0)

#define COL_PORTF_m (COL4_PIN_m | COL3_PIN_m)

#define PORTF_OUTPUT_m (COL_PORTF_m)

#endif //_PINDEFS_H_
