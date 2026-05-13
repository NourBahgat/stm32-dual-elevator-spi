#include "Dma.h"
#include "Rcc.h"
#include "stm32f401xe.h"

/* Minimal init: enable DMA2 clock and NVIC for Stream7 */
void Dma_Init(void)
{
    /* Enable DMA2 clock via RCC helper */
    Rcc_Enable(RCC_DMA2);

    /* Enable NVIC for DMA2 Stream7 (IRQ number from CMSIS) */
    NVIC_EnableIRQ(DMA2_Stream7_IRQn);
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
