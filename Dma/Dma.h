/* Dma.h
 * Minimal register-level DMA helpers for USART1 TX (DMA2 Stream7)
 */
#ifndef DMA_H
#define DMA_H

void Dma_Init(void);

/* DMA Stream Configuration Register (SxCR) Bit Definitions */
#ifndef DMA_SxCR_CHSEL_VAL
#define DMA_SxCR_CHSEL_VAL(v)   ((uint32)(v) << 25U)
#endif

#ifndef DMA_SxCR_DIR_M2P
#define DMA_SxCR_DIR_M2P        (1U << 6U)   /* Memory to Peripheral */
#endif

#ifndef DMA_SxCR_MINC
#define DMA_SxCR_MINC           (1U << 10U)  /* Memory Increment */
#endif

#ifndef DMA_SxCR_TCIE
#define DMA_SxCR_TCIE           (1U << 4U)   /* Transfer Complete Interrupt Enable */
#endif

#define DMA_SxCR_CLEAR_MASK     ( (7U << 25U) | (3U << 6U) | (3U << 16U) | (3U << 13U) | \
                                  (1U << 9U)  | (1U << 10U)| (1U << 8U)  | (3U << 11U) | \
                                  (1U << 18U) | (1U << 19U)| (1U << 20U) | (1U << 21U) )

#endif /* DMA_H */
