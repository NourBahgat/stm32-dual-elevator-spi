#ifndef NVIC_H
#define NVIC_H

#include "Std_Types.h"

void Nvic_EnableIrq(uint8 IrqNum);

void Nvic_DisableIrq(uint8 IrqNum);

void Nvic_SetPriority(uint8 IrqNum, uint8 Priority);

/* ── STM32F401 IRQ numbers (Table 38, RM0368) ──────────────────── */
#define EXTI0_IRQn      6U
#define EXTI1_IRQn      7U
#define EXTI2_IRQn      8U
#define EXTI3_IRQn      9U
#define EXTI4_IRQn     10U
#define EXTI9_5_IRQn   23U
#define TIM2_IRQn      28U
#define TIM3_IRQn      29U
#define TIM4_IRQn      30U
#define SPI1_IRQn      35U
#define USART1_IRQn    37U
#define EXTI15_10_IRQn 40U
#define TIM5_IRQn      50U

#endif /* NVIC_H */
