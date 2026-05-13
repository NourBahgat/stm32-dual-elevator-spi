#include "Dma.h"
#include "Rcc.h"
#include "stm32f401xe.h"
#include "Nvic.h"

/* Some build configurations may not expose the CMSIS IRQ name for DMA2
 * before this file is included. Provide a fallback numeric definition so
 * `Nvic_SetPriority()` can be called without requiring a specific include
 * order. STM32F4 DMA2 Stream7 global IRQ number is 70 (per CMSIS headers). */
#ifndef DMA2_Stream7_IRQn
#define DMA2_Stream7_IRQn 70U
#endif

/* Minimal init: enable DMA2 clock and NVIC for Stream7 */
void Dma_Init(void)
{
    /* Enable DMA2 clock via RCC helper */
    Rcc_Enable(RCC_DMA2);

    /* Set a safe priority for DMA IRQ and enable it using project NVIC API */
    Nvic_SetPriority(DMA2_Stream7_IRQn, 4U);
    Nvic_EnableIrq(DMA2_Stream7_IRQn);
}

/* DMA2 Stream7 IRQ handler — forward transfer-complete to telemetry */
void DMA2_Stream7_IRQHandler(void)
{
    /* Check transfer complete flag for stream7 */
    if (DMA2->HISR & DMA_HISR_TCIF7) {
        DMA2->HIFCR = DMA_HIFCR_CTCIF7;

        /* Telemetry layer provides completion callback in main.c */
        extern void Telemetry_DMA_CompleteCallback(void);
        Telemetry_DMA_CompleteCallback();
    }

    /* Clear any transfer error flags if set */
    if (DMA2->HISR & DMA_HISR_TEIF7) {
        DMA2->HIFCR = DMA_HIFCR_CTEIF7;
    }
}
