#include "Usart.h"
#include "Gpio.h"
#include "Nvic.h"
#include "RingBuffer.h"

/*
 * No stm32f401xe.h included — UsartType and USART1 are defined entirely
 * in Usart.h using a manual register map, consistent with the project's
 * driver architecture.
 */

/* =========================================================================
 * Module-private RX ring buffer
 * Populated by USART1_IRQHandler; consumed by Usart1_GetByte / ReadLine.
 * ========================================================================= */
static RingBufferType usart1_rx_ring;

/* Persistent line-assembly index for Usart1_ReadLine.
 * Static so it survives across calls (ring buffer may not have a full
 * line on the first call). */
static uint32 rx_line_idx = 0U;

/* =========================================================================
 * Usart1_Init
 * ========================================================================= */
void Usart1_Init(void) {

    /* ----- GPIO alternate function setup ---------------------------------
     *   PA9  = USART1_TX  (AF7)
     *   PA10 = USART1_RX  (AF7)
     * --------------------------------------------------------------------- */
    Gpio_Init(GPIO_A, 9U,  GPIO_AF, GPIO_PUSH_PULL);
    Gpio_Init(GPIO_A, 10U, GPIO_AF, GPIO_PUSH_PULL);

    Gpio_SetAF(GPIO_A, 9U,  GPIO_AF7);
    Gpio_SetAF(GPIO_A, 10U, GPIO_AF7);

    /* ----- Disable USART before configuration ---------------------------- */
    USART1->CR1 &= ~(1U << USART_CR1_UE_POS);

    /* ----- Word length: 8 bits (M = 0) ----------------------------------- */
    USART1->CR1 &= ~(1U << USART_CR1_M_POS);

    /* ----- Oversampling: 16x (OVER8 = 0) --------------------------------- */
    USART1->CR1 &= ~(1U << USART_CR1_OVER8_POS);

    /* ----- Stop bits: 1 (STOP[1:0] = 00) --------------------------------- */
    USART1->CR2 &= ~USART_CR2_STOP_MASK;

    /* ----- Parity: none (PCE = 0) ---------------------------------------- */
    USART1->CR1 &= ~(1U << USART_CR1_PCE_POS);

    /* ----- Baud rate: 115200 @ APB2 84 MHz ------------------------------- */
    USART1->BRR = USART_BRR_115200;

    /* ----- Enable transmitter and receiver -------------------------------- */
    USART1->CR1 |= (1U << USART_CR1_TE_POS);
    USART1->CR1 |= (1U << USART_CR1_RE_POS);

    /* ----- Enable RXNE interrupt ----------------------------------------- */
    USART1->CR1 |= (1U << USART_CR1_RXNEIE_POS);

    /* ----- Initialise ring buffer ---------------------------------------- */
    RB_Init(&usart1_rx_ring);

    /* ----- Enable USART1 IRQ in NVIC ------------------------------------- */
    Nvic_EnableIrq(USART1_IRQn);

    /* ----- Enable USART1 ------------------------------------------------- */
    USART1->CR1 |= (1U << USART_CR1_UE_POS);
}

/* =========================================================================
 * Usart1_TransmitByte
 *
 * Checks TXE once and loads the byte if the register is empty.
 * Does NOT wait for the transmission to complete (no while(TC)).
 *
 * The caller (Usart1_TransmitString) retries on Tx_NOK, which means
 * the CPU loops only for the ~1-byte-time it takes for TXE to clear —
 * this is the "polling allowed for UART TX" exception in the project spec.
 * ========================================================================= */
uint8 Usart1_TransmitByte(uint8 Byte) {
    if (USART1->SR & (1U << USART_SR_TXE_POS)) {
        USART1->DR = (uint32)Byte;
        return Tx_OK;
    }
    return Tx_NOK;
}

/* =========================================================================
 * Usart1_TransmitString
 *
 * Iterates through the string, retrying each byte until it is accepted.
 * No while(TC) anywhere — the CPU does not wait for a byte to physically
 * finish transmitting before loading the next one into DR.
 * ========================================================================= */
void Usart1_TransmitString(const char *Str) {
    uint32 i = 0U;

    while (Str[i] != '\0') {
        /* Retry the same byte until TXE is set.
         * TXE goes high as soon as DR is transferred to the shift register
         * (~1 bit-time), so this is never a long spin. */
        if (Usart1_TransmitByte((uint8)Str[i]) == Tx_OK) {
            i++;
        }
    }
}

/* =========================================================================
 * Usart1_GetByte
 *
 * Non-blocking single-byte read from the RX ring buffer.
 * Returns 1 if a byte was available, 0 if the buffer was empty.
 * ========================================================================= */
uint8 Usart1_GetByte(uint8 *Data) {
    return RB_Dequeue(&usart1_rx_ring, Data);
}

/* =========================================================================
 * Usart1_ReadLine
 *
 * Assembles a line from the ring buffer one character at a time.
 * Non-blocking: returns NULL on each call until a full line is ready.
 * Handles both '\n' and '\r\n' / '\n\r' line endings.
 *
 * The static rx_line_idx persists across calls so partial lines are
 * accumulated correctly between 500ms telemetry ticks.
 * ========================================================================= */
char *Usart1_ReadLine(char *Buffer, uint32 MaxLen) {
    uint8 ch;

    while (RB_Available(&usart1_rx_ring) > 0U) {

        ch = RB_Peek(&usart1_rx_ring);

        if (ch == '\n' || ch == '\r') {

            /* Consume the line terminator */
            RB_Dequeue(&usart1_rx_ring, &ch);

            /* Consume a paired second terminator (\r\n or \n\r) if present */
            if (RB_Available(&usart1_rx_ring) > 0U) {
                uint8 next = RB_Peek(&usart1_rx_ring);
                if ((ch == '\r' && next == '\n') ||
                    (ch == '\n' && next == '\r')) {
                    RB_Dequeue(&usart1_rx_ring, &next);
                }
            }

            /* Null-terminate and reset index for next line */
            Buffer[rx_line_idx] = '\0';
            rx_line_idx = 0U;
            return Buffer;

        } else {

            /* Regular character — consume and store */
            RB_Dequeue(&usart1_rx_ring, &ch);

            if (rx_line_idx < (MaxLen - 1U)) {
                Buffer[rx_line_idx] = (char)ch;
                rx_line_idx++;
            } else {
                /* Buffer full without finding a terminator.
                 * Discard accumulated data and signal overflow. */
                rx_line_idx = 0U;
                return (void *)0;
            }
        }
    }

    /* Ring buffer exhausted — no complete line yet */
    return (void *)0;
}

/* =========================================================================
 * USART1_IRQHandler
 *
 * Fires on RXNE (receive register not empty).
 * Reads DR — which simultaneously clears the RXNE flag — and stores
 * the byte in the ring buffer for later consumption by GetByte/ReadLine.
 *
 * Overrun error (ORE) is cleared here too: ORE prevents new bytes from
 * being received until SR and DR are both read, so clearing it keeps
 * the peripheral healthy under high-traffic conditions.
 * ========================================================================= */
void USART1_IRQHandler(void) {

    uint32 sr = USART1->SR;   /* Snapshot SR once */

    if (sr & (1U << USART_SR_RXNE_POS)) {
        uint8 byte = (uint8)USART1->DR;   /* Reading DR clears RXNE and ORE */
        RB_Enqueue(&usart1_rx_ring, byte);
    } else if (sr & (1U << USART_SR_ORE_POS)) {
        /* Overrun: dummy read of DR to clear ORE and unblock the receiver */
        (void)USART1->DR;
    }
}