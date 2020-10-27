// References
//
// #1. RM0090 Reference Manual - https://www.st.com/resource/en/reference_manual/dm00031020-stm32f405-415-stm32f407-417-stm32f427-437-and-stm32f429-439-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf
//     * STM32F40xxx Reset and clock control (RCC)
//
// #2. STM32F407xx Data Sheet  - https://www.st.com/resource/en/datasheet/dm00037051.pdf
//     * STM32F40xxx block diagram.
//
// #3. STM32F407xx User Manual - https://www.st.com/resource/en/user_manual/dm00039084-discovery-kit-with-stm32f407vg-mcu-stmicroelectronics.pdf
//     * STM32 pin description versus board functions.
//
// #4. Cortex M4 User Guide    - https://developer.arm.com/documentation/dui0553/latest/

// stm32f4xx.h contains structs and constants relating to the STM32F familly of microcontrollers.
// These encode the GPIO, peripherel registers and their memory-mapped offsets.
#include "stm32f4xx.h"

// Enable LEDs.
//
// 1. The LEDs are attached to 'GPIO PORT D' PD12 - PD15. (Ref: STM32F407xx User Manual - Tab. 7).
// 2. GPIO PORT D is attached to the 'AHB1' 168 MHz bus. (Ref: STM32F407xx Data Sheet - Fig. 5).
// 3. AHB1 PORT D Clock is enabled via the 'RCC AHB1 peripheral clock register (RCC_AHB1ENR)'. (Ref: RM0090 Reference Manual - Sec. 6.3.10).

// Registers
//
// 1. RCC AHB1 peripheral clock register (RCC_AHB1ENR) - Can be used to enable the RCC clock on AHB1.
//   * (Ref: RM0090 Reference Manual - Sec. 6.3.10).
//
// 2. GPIO port mode register (GPIOx_MODER) - Can be used to set the mode on GPIO ports.
//    * This is a 32 bit register, with two bits per mode, for 16 ports.
//    * (Ref: RM0090 Reference Manual - Sec. 8.4.1). 
//
// 3. GPIO port output data register (GPIOx_ODR) - Can be used to set the ouput pins on GPIO ports. 
//    * This is a 16 bit register, with one bits per mode, for 16 ports.
//    * (Ref: RM0090 Reference Manual - Sec. 8.4.6).


// RCC AHB1 peripheral clock register (RCC_AHB1ENR). (Ref: RM0090 Reference Manual - Sec. 6.3.10).
// * The register bit to enable the clock is Bit 3 'GPIODEN': IO port D clock enable.
// * Create a bit mask by shifting an unsigned int 3.
#define GPIOD_CLOCK (1U << 3)

// GPIO port mode register (GPIOx_MODER). (Ref: RM0090 Reference Manual - Sec. 8.4.1).
// * The mode register is 32 bits and serves 16 GPIO pins with two bits per pin (4 modes).
// * 'General Output Mode' is 01.
// * Create a bit mask for each PD 12-15. NB: This is double the PD as the mode register is 2 bits. 
#define GREEN_BIT (1U << 24)
#define ORANGE_BIT (1U << 26)
#define RED_BIT (1U << 28)
#define BLUE_BIT (1U << 30)

// GPIO port output data register (GPIOx_ODR). (Ref: RM0090 Reference Manual - Sec. 8.4.6).
// Create a bit mask by shifting an unsigned int the required number of positions.
// 
#define GREEN (1U << 12)
#define ORANGE (1U << 13)
#define RED (1U << 14)
#define BLUE (1U << 15)


int main() {
	// Reset and Clock Control - Set the RCC AHB1 peripheral clock register to enable the GPIOD port.
	RCC->AHB1ENR |= GPIOD_CLOCK;
	
	// GPIOD Port -  Set the MODER register to enable 
	GPIOD->MODER |= GREEN_BIT | ORANGE_BIT | RED_BIT | BLUE_BIT;
	
	while (1) {
		// GPIOD Port - set the GPIO port output data register to on.
		GPIOD->ODR |=  GREEN | ORANGE | RED | BLUE;
	}
}
