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
#define ENABLE_GPIOD_CLOCK (1U << 3)

// GPIO port mode register (GPIOx_MODER). (Ref: RM0090 Reference Manual - Sec. 8.4.1).
// * The mode register is 32 bits and serves 16 GPIO pins with two bits per pin (4 modes).
// * 'General Output Mode' is 01.
// * Create a bit mask for each PD 12-15. NB: This is double the PD as the mode register is 2 bits. 
#define ENABLE_GREEN_LED (1U << 24)
#define ENABLE_ORANGE_LED (1U << 26)
#define ENABLE_RED_LED (1U << 28)
#define BENABLE_BLUE_LED (1U << 30)

// GPIO port output data register (GPIOx_ODR). (Ref: RM0090 Reference Manual - Sec. 8.4.6).
// Create a bit mask by shifting an unsigned int the required number of positions.
// 
#define GREEN (1U << 12)
#define ORANGE (1U << 13)
#define RED (1U << 14)
#define BLUE (1U << 15)

// Volatile variables are variables that change without the code changing them, e.g. clock tick counter.
// The volatile keyword, stops the compiler optimising out the variable when it detects no programmatic changes.
//
// Whenever the SysTick interrupt is raised the tick will be incremented in the exception handler.
volatile uint32_t tick;
volatile uint32_t _tick;
	
void GPIO_Init(void);
void DelayMillis(uint32_t millis);

enum LEDState {
	ON, OFF, TOGGLE
};

void setAll(enum LEDState state);
void setBlue(enum LEDState state);
void setOrange(enum LEDState state);
void setRed(enum LEDState state);
void setGreen(enum LEDState state);

int green_main(void);
int blue_main(void);

// uVision - 'Options for Target' -> Floating Point Hardware : Not used. 

// main              - is the 'forground'.
// SysTick_Handler[] - is 'interupt service handler/routine' in the 'background'.
int main() {
	GPIO_Init();
	// stop the compiler optimising out 'green_main'.
	uint32_t volatile start = 0U;
	if (start) {
		blue_main();
	} else {
		green_main();
	}
	while(1) {}
	/*
	while (1) {
		setBlue(ON);
		DelayMillis(500);
		setBlue(OFF);
		DelayMillis(500);
	}
	*/
}

int blue_main() {
	// The while loop is what makes it like a thread / main function.
	while (1) {
		setBlue(ON);
		DelayMillis(500);
		setBlue(OFF);
		DelayMillis(500);
	}
}

int green_main() {
	// The while loop is what makes it like a thread / main function.
	while (1) {
		setGreen(ON);
		DelayMillis(500);
		setGreen(OFF);
		DelayMillis(500);
	}
}

void setAll(enum LEDState state) {
	switch (state) {
		case ON:
			// GPIOD Port - set the GPIO port output data register to on.
			GPIOD->ODR |= GREEN | ORANGE | RED | BLUE;
			break;
		case OFF:
			// GPIOD Port - set the GPIO port output data register to off.
			GPIOD->ODR &= ~GREEN | ORANGE | RED | BLUE;
			break;
		case TOGGLE:
			GPIOD->ODR ^= GREEN | ORANGE | RED | BLUE;; 
			break;
	}
}

void setBlue(enum LEDState state) {
	switch (state) {
		case ON:
			GPIOD->ODR |= BLUE;
			break;
		case OFF:
			GPIOD->ODR &= ~BLUE; 
			break;
		case TOGGLE:
			GPIOD->ODR ^= BLUE; 
			break;
	}
}

void setOrange(enum LEDState state) {
	switch (state) {
		case ON:
			GPIOD->ODR |= ORANGE;
			break;
		case OFF:
			GPIOD->ODR &= ~ORANGE; 
			break;
		case TOGGLE:
			GPIOD->ODR ^= ORANGE; 
			break;
	}
}

void setRed(enum LEDState state) {
	switch (state) {
		case ON:
			GPIOD->ODR |= RED;
			break;
		case OFF:
			GPIOD->ODR &= ~RED; 
			break;
		case TOGGLE:
			GPIOD->ODR ^= RED; 
			break;
	}
}

void setGreen(enum LEDState state) {
	switch (state) {
		case ON:
			GPIOD->ODR |= GREEN;
			break;
		case OFF:
			GPIOD->ODR &= ~GREEN; 
			break;
		case TOGGLE:
			GPIOD->ODR ^= GREEN; 
			break;
	}
}


void GPIO_Init() {
	// Enable Registers
	
	// Reset and Clock Control - Set the RCC AHB1 peripheral clock register to enable the GPIOD port.
	RCC->AHB1ENR |= ENABLE_GPIO_CLOCK;
	// GPIOD Port -  Set the MODER register to enable 
	GPIOD->MODER |= ENABLE_GREEN_LED | ENABLE_ORANGE_LED | ENABLE_RED_LED | BENABLE_BLUE_LED;
	
	// Enable CMSIS SysTick clock interrupts
	
	// Add a time-base with SysTick. SysTick is available through the CMSIS hardware abstraction layer.
	// The cortex M4 runs in the MHz range, so divding by 1000U causes clock interupts to occur every millisecond.
	// https://www.keil.com/pack/doc/CMSIS/Core/html/group__SysTick__gr.html
	SysTick_Config(SystemCoreClock/1000U);
	// Enable interupts.
	__enable_irq();
}


// This is 'interupt service handler/routine'. It run in the 'background'.
//
// We can break-point here to manually make a context switch using the debugger.
//
// 1. The PC register 'stack pointer' contains a reference to the memory location of the next instruction.
// 2. The SP register 'stack pointer' contains a reference to the memory location of the current stack..
//
// When an interupt occurs on a Cortex M processor, the content of some registers are saved onto the stack.
// The stack is a memory region in the RAM. These can be used re-instate the originl execution when the interupt
// service handler has finished executing.
//
// When the processor takes an exception, unless the exception is a tail-chained or a late-arriving
// exception, the processor pushes information onto the current stack. This operation is referred to
// as 'stacking' and the structure of eight data words is referred as the 'stack frame'.
// (Ref: Cortex M4 User Guide - Sec. 2.3.7)
//
// We can break point here, and, update the captured stack frame PC and set it to the desired LED main function.
//
// NB: This function is also implemented by the STM32 HAL libraries, along with delay functions, etc.
void SysTick_Handler() {
	++tick;
}

uint32_t getTick(void) {
	__disable_irq();
	_tick = tick; // Critical Section - Multiple assembler instructions. To ensure it executes atomically, interrupts are temporarily disabled.
	__enable_irq();	 
	return _tick;
}

// This code is 'blocking'. It just eats up time doing nothing.
void DelayMillis(uint32_t millis) {
	uint32_t temp = getTick();
	while ((getTick() - temp) < millis) {}
	
}
