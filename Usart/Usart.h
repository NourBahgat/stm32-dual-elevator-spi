#ifndef USART_H
#define USART_H

#include "Std_Types.h"

/* =========================================================================
 * USART1 Register Map  (STM32F401 Reference Manual, Section 19.6)
 * Base address: 0x40011000
 * ========================================================================= */
typedef struct {
    volatile uint32 SR;       /* 0x00  Status register                  */
    volatile uint32 DR;       /* 0x04  Data register                    */
    volatile uint32 BRR;      /* 0x08  Baud rate register               */
    volatile uint32 CR1;      /* 0x0C  Control register 1               */
    volatile uint32 CR2;      /* 0x10  Control register 2               */
    volatile uint32 CR3;      /* 0x14  Control register 3               */
    volatile uint32 GTPR;     /* 0x18  Guard time and prescaler register */
} UsartType;

#define USART1_BASE_ADDR   0x40011000UL
#define USART1             ((UsartType *)USART1_BASE_ADDR)

/* =========================================================================
 * SR — Status Register bit positions
 * ========================================================================= */
#define USART_SR_PE_POS      0U   /* Parity error          */
#define USART_SR_FE_POS      1U   /* Framing error         */
#define USART_SR_NF_POS      2U   /* Noise detected        */
#define USART_SR_ORE_POS     3U   /* Overrun error         */
#define USART_SR_IDLE_POS    4U   /* IDLE line detected    */
#define USART_SR_RXNE_POS    5U   /* Read data reg not empty */
#define USART_SR_TC_POS      6U   /* Transmission complete */
#define USART_SR_TXE_POS     7U   /* Transmit data reg empty */

/* =========================================================================
 * CR1 — Control Register 1 bit positions
 * ========================================================================= */
#define USART_CR1_SBK_POS    0U   /* Send break                    */
#define USART_CR1_RWU_POS    1U   /* Receiver wakeup               */
#define USART_CR1_RE_POS     2U   /* Receiver enable               */
#define USART_CR1_TE_POS     3U   /* Transmitter enable            */
#define USART_CR1_IDLEIE_POS 4U   /* IDLE interrupt enable         */
#define USART_CR1_RXNEIE_POS 5U   /* RXNE interrupt enable         */
#define USART_CR1_TCIE_POS   6U   /* Transmission complete IE      */
#define USART_CR1_TXEIE_POS  7U   /* TXE interrupt enable          */
#define USART_CR1_PEIE_POS   8U   /* PE interrupt enable           */
#define USART_CR1_PS_POS     9U   /* Parity selection              */
#define USART_CR1_PCE_POS    10U  /* Parity control enable         */
#define USART_CR1_WAKE_POS   11U  /* Wakeup method                 */
#define USART_CR1_M_POS      12U  /* Word length (0=8bit, 1=9bit)  */
#define USART_CR1_UE_POS     13U  /* USART enable                  */
#define USART_CR1_OVER8_POS  15U  /* Oversampling mode             */

/* =========================================================================
 * CR2 — Control Register 2 bit positions (stop bits field)
 * ========================================================================= */
#define USART_CR2_STOP_POS   12U  /* STOP[13:12] bits */
#define USART_CR2_STOP_MASK  (0x3U << USART_CR2_STOP_POS)

/* =========================================================================
 * BRR value for target baud rates
 * Formula:  BRR = fCLK / (16 * BaudRate)
 * USART1 is on APB2 = 16 MHz (HSI default)
 *
 *  9600   baud → BRR = 16,000,000 / (16 * 9600)   = 104.16 → 0x683
 *  115200 baud → BRR = 16,000,000 / (16 * 115,200) = 8.68   → 0x8B
 * ========================================================================= */
#define USART_BRR_9600       0x0683U   /* 9600   baud @ 16 MHz */
#define USART_BRR_115200     0x008BU   /* 115200 baud @ 16 MHz */

/* =========================================================================
 * Ring buffer size — must be a power of 2
 * ========================================================================= */
#define USART_RX_BUF_SIZE    64U

/* =========================================================================
 * Return codes
 * ========================================================================= */
#define Tx_OK                0U
#define Tx_NOK               1U

/* =========================================================================
 * Public API
 * ========================================================================= */

/**
 * @brief  Initialise USART1 peripheral and GPIO alternate functions.
 *         GPIO: PA9 = TX, PA10 = RX (AF7).
 *         Configuration: 8-N-1, 115200 baud, 16x oversampling.
 *         RX is interrupt-driven (RXNEIE); TX uses non-blocking flag check.
 */
void Usart1_Init(void);

/**
 * @brief  Attempt to transmit one byte.
 *         Checks TXE once — does NOT wait for transmission to complete.
 *         Returns Tx_NOK if the TX register is still busy; caller must retry.
 *
 * @param  Byte  Byte to transmit.
 * @return Tx_OK if byte was loaded into DR, Tx_NOK if TXE was not set.
 */
uint8 Usart1_TransmitByte(uint8 Byte);

/**
 * @brief  Transmit a null-terminated string.
 *         Retries each byte until TXE is set — does NOT block on TC.
 *         Acceptable for the 500ms telemetry use-case per project spec.
 *
 * @param  Str  Null-terminated ASCII string.
 */
void Usart1_TransmitString(const char *Str);

/**
 * @brief  Non-blocking read of a single byte from the RX ring buffer.
 *
 * @param  Data  Output pointer; set to received byte if available.
 * @return 1 if a byte was available and written to *Data, 0 otherwise.
 */
uint8 Usart1_GetByte(uint8 *Data);

/**
 * @brief  Extract a complete '\n' or '\r' terminated line from the ring buffer.
 *         Non-blocking — returns NULL if no full line is available yet.
 *         The returned string is null-terminated; maxLen includes the '\0'.
 *
 * @param  Buffer  Caller-supplied buffer of at least maxLen bytes.
 * @param  MaxLen  Maximum number of characters including null terminator.
 * @return Pointer to Buffer if a line was extracted, NULL otherwise.
 */
char *Usart1_ReadLine(char *Buffer, uint32 MaxLen);

/**
 * @brief  USART1 global interrupt handler.
 *         Handles RXNE: reads DR into the ring buffer.
 *         Must be placed in the vector table as USART1_IRQHandler.
 */
void USART1_IRQHandler(void);

/* Non-blocking DMA transmit API (register-level DMA2 Stream7 for USART1 TX) */
void Usart1_TransmitDMA(uint8 *data, uint16 len);

#endif /* USART_H */