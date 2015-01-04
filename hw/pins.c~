#include "pins.h"

#define GPIO_BITBAND_ADDR(reg, bit) (((uint32_t)&(reg) - 0x40000000) * 32 + (bit) * 4 + 0x42000000)
#define GPIO_BITBAND_PTR(reg, bit) ((uint32_t *)GPIO_BITBAND_ADDR((reg), (bit)))

struct digital_pin_bitband_and_config_table_struct {
        volatile uint32_t *reg;
        volatile uint32_t *config;
};

const struct digital_pin_bitband_and_config_table_struct digital_pin_to_info_PGM[] = {
	{GPIO_BITBAND_PTR(CORE_PIN0_PORTREG, CORE_PIN0_BIT), &CORE_PIN0_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN1_PORTREG, CORE_PIN1_BIT), &CORE_PIN1_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN2_PORTREG, CORE_PIN2_BIT), &CORE_PIN2_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN3_PORTREG, CORE_PIN3_BIT), &CORE_PIN3_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN4_PORTREG, CORE_PIN4_BIT), &CORE_PIN4_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN5_PORTREG, CORE_PIN5_BIT), &CORE_PIN5_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN6_PORTREG, CORE_PIN6_BIT), &CORE_PIN6_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN7_PORTREG, CORE_PIN7_BIT), &CORE_PIN7_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN8_PORTREG, CORE_PIN8_BIT), &CORE_PIN8_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN9_PORTREG, CORE_PIN9_BIT), &CORE_PIN9_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN10_PORTREG, CORE_PIN10_BIT), &CORE_PIN10_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN11_PORTREG, CORE_PIN11_BIT), &CORE_PIN11_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN12_PORTREG, CORE_PIN12_BIT), &CORE_PIN12_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN13_PORTREG, CORE_PIN13_BIT), &CORE_PIN13_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN14_PORTREG, CORE_PIN14_BIT), &CORE_PIN14_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN15_PORTREG, CORE_PIN15_BIT), &CORE_PIN15_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN16_PORTREG, CORE_PIN16_BIT), &CORE_PIN16_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN17_PORTREG, CORE_PIN17_BIT), &CORE_PIN17_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN18_PORTREG, CORE_PIN18_BIT), &CORE_PIN18_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN19_PORTREG, CORE_PIN19_BIT), &CORE_PIN19_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN20_PORTREG, CORE_PIN20_BIT), &CORE_PIN20_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN21_PORTREG, CORE_PIN21_BIT), &CORE_PIN21_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN22_PORTREG, CORE_PIN22_BIT), &CORE_PIN22_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN23_PORTREG, CORE_PIN23_BIT), &CORE_PIN23_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN24_PORTREG, CORE_PIN24_BIT), &CORE_PIN24_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN25_PORTREG, CORE_PIN25_BIT), &CORE_PIN25_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN26_PORTREG, CORE_PIN26_BIT), &CORE_PIN26_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN27_PORTREG, CORE_PIN27_BIT), &CORE_PIN27_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN28_PORTREG, CORE_PIN28_BIT), &CORE_PIN28_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN29_PORTREG, CORE_PIN29_BIT), &CORE_PIN29_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN30_PORTREG, CORE_PIN30_BIT), &CORE_PIN30_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN31_PORTREG, CORE_PIN31_BIT), &CORE_PIN31_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN32_PORTREG, CORE_PIN32_BIT), &CORE_PIN32_CONFIG},
	{GPIO_BITBAND_PTR(CORE_PIN33_PORTREG, CORE_PIN33_BIT), &CORE_PIN33_CONFIG}
};






inline void pset(uint8_t pin, uint8_t val) __attribute__((always_inline, unused));
inline void pset(uint8_t pin, uint8_t val){
	if (__builtin_constant_p(pin)) {
		if (val) {
			if (pin == 0) {
				CORE_PIN0_PORTSET = CORE_PIN0_BITMASK;
			} else if (pin == 1) {
				CORE_PIN1_PORTSET = CORE_PIN1_BITMASK;
			} else if (pin == 2) {
				CORE_PIN2_PORTSET = CORE_PIN2_BITMASK;
			} else if (pin == 3) {
				CORE_PIN3_PORTSET = CORE_PIN3_BITMASK;
			} else if (pin == 4) {
				CORE_PIN4_PORTSET = CORE_PIN4_BITMASK;
			} else if (pin == 5) {
				CORE_PIN5_PORTSET = CORE_PIN5_BITMASK;
			} else if (pin == 6) {
				CORE_PIN6_PORTSET = CORE_PIN6_BITMASK;
			} else if (pin == 7) {
				CORE_PIN7_PORTSET = CORE_PIN7_BITMASK;
			} else if (pin == 8) {
				CORE_PIN8_PORTSET = CORE_PIN8_BITMASK;
			} else if (pin == 9) {
				CORE_PIN9_PORTSET = CORE_PIN9_BITMASK;
			} else if (pin == 10) {
				CORE_PIN10_PORTSET = CORE_PIN10_BITMASK;
			} else if (pin == 11) {
				CORE_PIN11_PORTSET = CORE_PIN11_BITMASK;
			} else if (pin == 12) {
				CORE_PIN12_PORTSET = CORE_PIN12_BITMASK;
			} else if (pin == 13) {
				CORE_PIN13_PORTSET = CORE_PIN13_BITMASK;
			} else if (pin == 14) {
				CORE_PIN14_PORTSET = CORE_PIN14_BITMASK;
			} else if (pin == 15) {
				CORE_PIN15_PORTSET = CORE_PIN15_BITMASK;
			} else if (pin == 16) {
				CORE_PIN16_PORTSET = CORE_PIN16_BITMASK;
			} else if (pin == 17) {
				CORE_PIN17_PORTSET = CORE_PIN17_BITMASK;
			} else if (pin == 18) {
				CORE_PIN18_PORTSET = CORE_PIN18_BITMASK;
			} else if (pin == 19) {
				CORE_PIN19_PORTSET = CORE_PIN19_BITMASK;
			} else if (pin == 20) {
				CORE_PIN20_PORTSET = CORE_PIN20_BITMASK;
			} else if (pin == 21) {
				CORE_PIN21_PORTSET = CORE_PIN21_BITMASK;
			} else if (pin == 22) {
				CORE_PIN22_PORTSET = CORE_PIN22_BITMASK;
			} else if (pin == 23) {
				CORE_PIN23_PORTSET = CORE_PIN23_BITMASK;
			} else if (pin == 24) {
				CORE_PIN24_PORTSET = CORE_PIN24_BITMASK;
			} else if (pin == 25) {
				CORE_PIN25_PORTSET = CORE_PIN25_BITMASK;
			} else if (pin == 26) {
				CORE_PIN26_PORTSET = CORE_PIN26_BITMASK;
			} else if (pin == 27) {
				CORE_PIN27_PORTSET = CORE_PIN27_BITMASK;
			} else if (pin == 28) {
				CORE_PIN28_PORTSET = CORE_PIN28_BITMASK;
			} else if (pin == 29) {
				CORE_PIN29_PORTSET = CORE_PIN29_BITMASK;
			} else if (pin == 30) {
				CORE_PIN30_PORTSET = CORE_PIN30_BITMASK;
			} else if (pin == 31) {
				CORE_PIN31_PORTSET = CORE_PIN31_BITMASK;
			} else if (pin == 32) {
				CORE_PIN32_PORTSET = CORE_PIN32_BITMASK;
			} else if (pin == 33) {
				CORE_PIN33_PORTSET = CORE_PIN33_BITMASK;
			}
		} else {
			if (pin == 0) {
				CORE_PIN0_PORTCLEAR = CORE_PIN0_BITMASK;
			} else if (pin == 1) {
				CORE_PIN1_PORTCLEAR = CORE_PIN1_BITMASK;
			} else if (pin == 2) {
				CORE_PIN2_PORTCLEAR = CORE_PIN2_BITMASK;
			} else if (pin == 3) {
				CORE_PIN3_PORTCLEAR = CORE_PIN3_BITMASK;
			} else if (pin == 4) {
				CORE_PIN4_PORTCLEAR = CORE_PIN4_BITMASK;
			} else if (pin == 5) {
				CORE_PIN5_PORTCLEAR = CORE_PIN5_BITMASK;
			} else if (pin == 6) {
				CORE_PIN6_PORTCLEAR = CORE_PIN6_BITMASK;
			} else if (pin == 7) {
				CORE_PIN7_PORTCLEAR = CORE_PIN7_BITMASK;
			} else if (pin == 8) {
				CORE_PIN8_PORTCLEAR = CORE_PIN8_BITMASK;
			} else if (pin == 9) {
				CORE_PIN9_PORTCLEAR = CORE_PIN9_BITMASK;
			} else if (pin == 10) {
				CORE_PIN10_PORTCLEAR = CORE_PIN10_BITMASK;
			} else if (pin == 11) {
				CORE_PIN11_PORTCLEAR = CORE_PIN11_BITMASK;
			} else if (pin == 12) {
				CORE_PIN12_PORTCLEAR = CORE_PIN12_BITMASK;
			} else if (pin == 13) {
				CORE_PIN13_PORTCLEAR = CORE_PIN13_BITMASK;
			} else if (pin == 14) {
				CORE_PIN14_PORTCLEAR = CORE_PIN14_BITMASK;
			} else if (pin == 15) {
				CORE_PIN15_PORTCLEAR = CORE_PIN15_BITMASK;
			} else if (pin == 16) {
				CORE_PIN16_PORTCLEAR = CORE_PIN16_BITMASK;
			} else if (pin == 17) {
				CORE_PIN17_PORTCLEAR = CORE_PIN17_BITMASK;
			} else if (pin == 18) {
				CORE_PIN18_PORTCLEAR = CORE_PIN18_BITMASK;
			} else if (pin == 19) {
				CORE_PIN19_PORTCLEAR = CORE_PIN19_BITMASK;
			} else if (pin == 20) {
				CORE_PIN20_PORTCLEAR = CORE_PIN20_BITMASK;
			} else if (pin == 21) {
				CORE_PIN21_PORTCLEAR = CORE_PIN21_BITMASK;
			} else if (pin == 22) {
				CORE_PIN22_PORTCLEAR = CORE_PIN22_BITMASK;
			} else if (pin == 23) {
				CORE_PIN23_PORTCLEAR = CORE_PIN23_BITMASK;
			} else if (pin == 24) {
				CORE_PIN24_PORTCLEAR = CORE_PIN24_BITMASK;
			} else if (pin == 25) {
				CORE_PIN25_PORTCLEAR = CORE_PIN25_BITMASK;
			} else if (pin == 26) {
				CORE_PIN26_PORTCLEAR = CORE_PIN26_BITMASK;
			} else if (pin == 27) {
				CORE_PIN27_PORTCLEAR = CORE_PIN27_BITMASK;
			} else if (pin == 28) {
				CORE_PIN28_PORTCLEAR = CORE_PIN28_BITMASK;
			} else if (pin == 29) {
				CORE_PIN29_PORTCLEAR = CORE_PIN29_BITMASK;
			} else if (pin == 30) {
				CORE_PIN30_PORTCLEAR = CORE_PIN30_BITMASK;
			} else if (pin == 31) {
				CORE_PIN31_PORTCLEAR = CORE_PIN31_BITMASK;
			} else if (pin == 32) {
				CORE_PIN32_PORTCLEAR = CORE_PIN32_BITMASK;
			} else if (pin == 33) {
				CORE_PIN33_PORTCLEAR = CORE_PIN33_BITMASK;
			}
		}
	} else {
		if (val) {
			*portSetRegister(pin) = 1;
		} else {
			*portClearRegister(pin) = 1;
		}
	}
}

inline uint8_t pread(uint8_t pin) __attribute__((always_inline, unused));
inline uint8_t pread(uint8_t pin){
	if (__builtin_constant_p(pin)) {
		if (pin == 0) {
			return (CORE_PIN0_PINREG & CORE_PIN0_BITMASK) ? 1 : 0;
		} else if (pin == 1) {
			return (CORE_PIN1_PINREG & CORE_PIN1_BITMASK) ? 1 : 0;
		} else if (pin == 2) {
			return (CORE_PIN2_PINREG & CORE_PIN2_BITMASK) ? 1 : 0;
		} else if (pin == 3) {
			return (CORE_PIN3_PINREG & CORE_PIN3_BITMASK) ? 1 : 0;
		} else if (pin == 4) {
			return (CORE_PIN4_PINREG & CORE_PIN4_BITMASK) ? 1 : 0;
		} else if (pin == 5) {
			return (CORE_PIN5_PINREG & CORE_PIN5_BITMASK) ? 1 : 0;
		} else if (pin == 6) {
			return (CORE_PIN6_PINREG & CORE_PIN6_BITMASK) ? 1 : 0;
		} else if (pin == 7) {
			return (CORE_PIN7_PINREG & CORE_PIN7_BITMASK) ? 1 : 0;
		} else if (pin == 8) {
			return (CORE_PIN8_PINREG & CORE_PIN8_BITMASK) ? 1 : 0;
		} else if (pin == 9) {
			return (CORE_PIN9_PINREG & CORE_PIN9_BITMASK) ? 1 : 0;
		} else if (pin == 10) {
			return (CORE_PIN10_PINREG & CORE_PIN10_BITMASK) ? 1 : 0;
		} else if (pin == 11) {
			return (CORE_PIN11_PINREG & CORE_PIN11_BITMASK) ? 1 : 0;
		} else if (pin == 12) {
			return (CORE_PIN12_PINREG & CORE_PIN12_BITMASK) ? 1 : 0;
		} else if (pin == 13) {
			return (CORE_PIN13_PINREG & CORE_PIN13_BITMASK) ? 1 : 0;
		} else if (pin == 14) {
			return (CORE_PIN14_PINREG & CORE_PIN14_BITMASK) ? 1 : 0;
		} else if (pin == 15) {
			return (CORE_PIN15_PINREG & CORE_PIN15_BITMASK) ? 1 : 0;
		} else if (pin == 16) {
			return (CORE_PIN16_PINREG & CORE_PIN16_BITMASK) ? 1 : 0;
		} else if (pin == 17) {
			return (CORE_PIN17_PINREG & CORE_PIN17_BITMASK) ? 1 : 0;
		} else if (pin == 18) {
			return (CORE_PIN18_PINREG & CORE_PIN18_BITMASK) ? 1 : 0;
		} else if (pin == 19) {
			return (CORE_PIN19_PINREG & CORE_PIN19_BITMASK) ? 1 : 0;
		} else if (pin == 20) {
			return (CORE_PIN20_PINREG & CORE_PIN20_BITMASK) ? 1 : 0;
		} else if (pin == 21) {
			return (CORE_PIN21_PINREG & CORE_PIN21_BITMASK) ? 1 : 0;
		} else if (pin == 22) {
			return (CORE_PIN22_PINREG & CORE_PIN22_BITMASK) ? 1 : 0;
		} else if (pin == 23) {
			return (CORE_PIN23_PINREG & CORE_PIN23_BITMASK) ? 1 : 0;
		} else if (pin == 24) {
			return (CORE_PIN24_PINREG & CORE_PIN24_BITMASK) ? 1 : 0;
		} else if (pin == 25) {
			return (CORE_PIN25_PINREG & CORE_PIN25_BITMASK) ? 1 : 0;
		} else if (pin == 26) {
			return (CORE_PIN26_PINREG & CORE_PIN26_BITMASK) ? 1 : 0;
		} else if (pin == 27) {
			return (CORE_PIN27_PINREG & CORE_PIN27_BITMASK) ? 1 : 0;
		} else if (pin == 28) {
			return (CORE_PIN28_PINREG & CORE_PIN28_BITMASK) ? 1 : 0;
		} else if (pin == 29) {
			return (CORE_PIN29_PINREG & CORE_PIN29_BITMASK) ? 1 : 0;
		} else if (pin == 30) {
			return (CORE_PIN30_PINREG & CORE_PIN30_BITMASK) ? 1 : 0;
		} else if (pin == 31) {
			return (CORE_PIN31_PINREG & CORE_PIN31_BITMASK) ? 1 : 0;
		} else if (pin == 32) {
			return (CORE_PIN32_PINREG & CORE_PIN32_BITMASK) ? 1 : 0;
		} else if (pin == 33) {
			return (CORE_PIN33_PINREG & CORE_PIN33_BITMASK) ? 1 : 0;
		} else {
			return 0;
		}
	} else {
		return *portInputRegister(pin);
	}
}


inline void delay_us(uint32_t) __attribute__((always_inline, unused));
inline void delay_us(uint32_t usec){
#if F_CPU == 96000000
	uint32_t n = usec << 5;
#elif F_CPU == 48000000
	uint32_t n = usec << 4;
#elif F_CPU == 24000000
	uint32_t n = usec << 3;
#endif
	if (usec == 0) return;
	asm volatile(
		"L_%=_delayMicroseconds:"		"\n\t"
		"subs   %0, #1"				"\n\t"
		"bne    L_%=_delayMicroseconds"		"\n"
		: "+r" (n) :
	);
}


void pmode(uint8_t pin, uint8_t mode){
	volatile uint32_t *config;

	if (pin >= CORE_NUM_DIGITAL) return;
	config = portConfigRegister(pin);

	if (mode == OUTPUT) {
		*portModeRegister(pin) = 1;
		*config = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
	} else {
		*portModeRegister(pin) = 0;
		if (mode == INPUT) {
			*config = PORT_PCR_MUX(1);
		} else {
			*config = PORT_PCR_MUX(1) | PORT_PCR_PE | PORT_PCR_PS; // pullup
		}
	}
}

