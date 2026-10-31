/* SPDX-License-Identifier: AGPL-3.0-or-later */
/* Bare-metal startup and vector table for Nordic nRF54L15 Application Core */

#include <stdint.h>
#include "nrf.h"
#include "nrf54l/system_nrf54l.h"

extern uint32_t _estack;
extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;

extern int main(void);

void Default_Handler(void) {
    while (1) {
        __NOP();
    }
}

void Reset_Handler(void);
void NMI_Handler(void)        __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void)  __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler(void)  __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SecureFault_Handler(void)__attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void)        __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void)    __attribute__((weak, alias("Default_Handler")));

/* Peripheral IRQ handlers used by HAL */
void SPIM00_IRQHandler(void)     __attribute__((weak, alias("Default_Handler")));
void TIMER10_IRQHandler(void)    __attribute__((weak, alias("Default_Handler")));
void UARTE20_IRQHandler(void)    __attribute__((weak, alias("Default_Handler")));
void UARTE21_IRQHandler(void)    __attribute__((weak, alias("Default_Handler")));
void UARTE22_IRQHandler(void)    __attribute__((weak, alias("Default_Handler")));
void TIMER20_IRQHandler(void)    __attribute__((weak, alias("Default_Handler")));
void GPIOTE20_0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void GPIOTE20_1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));

typedef void (*vector_fn_t)(void);

__attribute__((section(".isr_vector"), used))
const vector_fn_t __Vectors[16 + 270] = {
    [0]  = (vector_fn_t)(uintptr_t)&_estack,
    [1]  = Reset_Handler,
    [2]  = NMI_Handler,
    [3]  = HardFault_Handler,
    [4]  = MemManage_Handler,
    [5]  = BusFault_Handler,
    [6]  = UsageFault_Handler,
    [7]  = SecureFault_Handler,
    [11] = SVC_Handler,
    [12] = DebugMon_Handler,
    [14] = PendSV_Handler,
    [15] = SysTick_Handler,

    /* External Interrupts (IRQn + 16) */
    [16 + 74]  = SPIM00_IRQHandler,     /* SPIM00 / UARTE00 */
    [16 + 133] = TIMER10_IRQHandler,    /* TIMER10 */
    [16 + 198] = UARTE20_IRQHandler,    /* SPIM20 / TWIM20 / UARTE20 */
    [16 + 199] = UARTE21_IRQHandler,    /* SPIM21 / TWIM21 / UARTE21 */
    [16 + 200] = UARTE22_IRQHandler,    /* SPIM22 / TWIM22 / UARTE22 */
    [16 + 202] = TIMER20_IRQHandler,    /* TIMER20 */
    [16 + 218] = GPIOTE20_0_IRQHandler, /* GPIOTE20_0 */
    [16 + 219] = GPIOTE20_1_IRQHandler, /* GPIOTE20_1 */
};

void Reset_Handler(void) {
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;
    while (dst < &_edata) {
        *dst++ = *src++;
    }

    dst = &_sbss;
    while (dst < &_ebss) {
        *dst++ = 0;
    }

    SCB->VTOR = (uint32_t)(uintptr_t)__Vectors;

    /* Enable FPU */
    SCB->CPACR |= (3UL << 20) | (3UL << 22);
    __DSB();
    __ISB();

    SystemInit();
    SystemCoreClockUpdate();

    (void)main();
    while (1) {
        __WFE();
    }
}
