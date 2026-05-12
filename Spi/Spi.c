#include "Gpio.h"
#include "Spi.h"

/*
 * No stm32f401xe.h included here — SpiType and SPI1 are defined
 * entirely in Spi.h using the manual register map.
 */

/* =========================================================================
 * Module-private state
 * All flags shared with SPI1_IRQHandler must be volatile.
 * ========================================================================= */
static volatile uint8           *spi_TxBuf    = (void *)0;
static volatile uint8           *spi_RxBuf    = (void *)0;
static volatile uint8            spi_Index    = 0U;
static volatile Spi_StatusType   spi_Status   = SPI_STATUS_IDLE;
static          Spi_FrameCallback spi_Callback = (void *)0;

/* =========================================================================
 * Private helpers: enable / disable the two data interrupts
 * ========================================================================= */
static void prv_EnableDataIrqs(void) {
    /* Enable RXNE first so we never miss the echo of the byte already
     * written to DR in Spi1_StartTransfer. */
    SPI1->CR2 |= (1U << SPI_CR2_RXNEIE_POS);
    SPI1->CR2 |= (1U << SPI_CR2_TXEIE_POS);
}

static void prv_DisableDataIrqs(void) {
    SPI1->CR2 &= ~(1U << SPI_CR2_TXEIE_POS);
    SPI1->CR2 &= ~(1U << SPI_CR2_RXNEIE_POS);
}

/* =========================================================================
 * Spi1_Init
 * ========================================================================= */
void Spi1_Init(uint8 MasterSlave, uint8 ClkPol, uint8 ClkPhase) {

    /* ----- GPIO alternate function setup ---------------------------------
     *   PB3 = SCK  |  PB4 = MISO  |  PB5 = MOSI   (AF5 on STM32F401)
     * --------------------------------------------------------------------- */
    Gpio_Init(GPIO_B, 3U, GPIO_AF, GPIO_PUSH_PULL);
    Gpio_Init(GPIO_B, 4U, GPIO_AF, GPIO_PUSH_PULL);
    Gpio_Init(GPIO_B, 5U, GPIO_AF, GPIO_PUSH_PULL);

    Gpio_SetAF(GPIO_B, 3U, GPIO_AF5);
    Gpio_SetAF(GPIO_B, 4U, GPIO_AF5);
    Gpio_SetAF(GPIO_B, 5U, GPIO_AF5);

    /* ----- Disable SPI before touching configuration bits ---------------- */
    SPI1->CR1 &= ~(1U << SPI_CR1_SPE_POS);

    /* ----- Software slave management: SSM=1, SSI=1 ----------------------- */
    SPI1->CR1 |= (1U << SPI_CR1_SSM_POS);
    SPI1->CR1 |= (1U << SPI_CR1_SSI_POS);

    /* ----- Master / Slave selection -------------------------------------- */
    SPI1->CR1 &= ~(1U << SPI_CR1_MSTR_POS);
    SPI1->CR1 |=  ((uint32)MasterSlave << SPI_CR1_MSTR_POS);

    /* ----- Clock polarity ------------------------------------------------ */
    SPI1->CR1 &= ~(1U << SPI_CR1_CPOL_POS);
    SPI1->CR1 |=  ((uint32)ClkPol << SPI_CR1_CPOL_POS);

    /* ----- Clock phase --------------------------------------------------- */
    SPI1->CR1 &= ~(1U << SPI_CR1_CPHA_POS);
    SPI1->CR1 |=  ((uint32)ClkPhase << SPI_CR1_CPHA_POS);

    /* ----- Baud rate ----------------------------------------------------- */
    SPI1->CR1 &= ~SPI_CR1_BR_MASK;
    SPI1->CR1 |=  ((uint32)SPI_BR_DIVIDER << SPI_CR1_BR_POS);

    /* ----- 8-bit data frame: DFF = 0 (default, but set explicitly) ------- */
    SPI1->CR1 &= ~SPI_CR1_DFF_MASK;

    /* ----- Enable SPI ---------------------------------------------------- */
    SPI1->CR1 |= (1U << SPI_CR1_SPE_POS);

    /* ----- Reset driver state -------------------------------------------- */
    spi_TxBuf    = (void *)0;
    spi_RxBuf    = (void *)0;
    spi_Index    = 0U;
    spi_Status   = SPI_STATUS_IDLE;
    spi_Callback = (void *)0;
}

/* =========================================================================
 * Spi1_RegisterCallback
 * ========================================================================= */
void Spi1_RegisterCallback(Spi_FrameCallback Callback) {
    spi_Callback = Callback;
}

/* =========================================================================
 * Spi1_StartTransfer
 *
 * Arms the ISR engine for one SPI_FRAME_SIZE full-duplex exchange.
 *
 * Key design point — pre-loading byte 0:
 *   The byte is already sitting in the shift
 *   register when the master asserts CS and starts clocking SCK, so there
 *   is zero slave-response latency. The same write also benefits the master
 *   by avoiding a one-IRQ-cycle gap before the first byte goes out.
 * ========================================================================= */
uint8 Spi1_StartTransfer(uint8 *TxBuffer, uint8 *RxBuffer) {

    if (spi_Status == SPI_STATUS_BUSY) {
        return SPI_NOK;   /* Previous transfer still in progress */
    }

    spi_TxBuf  = TxBuffer;
    spi_RxBuf  = RxBuffer;
    spi_Index  = 0U;
    spi_Status = SPI_STATUS_BUSY;

    /* Pre-load byte 0 into DR directly (TXE is always set when peripheral
     * is idle). spi_Index advances to 1 so the ISR continues from byte 1. */
    if (SPI1->SR & (1U << SPI_SR_TXE_POS)) {
        SPI1->DR  = (uint32)spi_TxBuf[0];
        spi_Index = 1U;
    }

    /* Arm interrupts — RXNE before TXEIE so the first echo cannot arrive
     * before we are ready to store it. */
    prv_EnableDataIrqs();

    return SPI_OK;
}

/* =========================================================================
 * Spi1_GetStatus
 * ========================================================================= */
Spi_StatusType Spi1_GetStatus(void) {
    return spi_Status;
}

/* =========================================================================
 * SPI1_IRQHandler
 *
 * Full-duplex SPI always exchanges bytes in lockstep: one TX byte clocked
 * out produces exactly one RX byte clocked in.  The strategy here is:
 *
 *   TX path  — fires on TXE; loads the next byte and increments spi_Index.
 *   RX path  — fires on RXNE; stores the received byte using (spi_Index - 1)
 *              because spi_Index already points at the *next* TX slot.
 *
 * When spi_Index reaches SPI_FRAME_SIZE after the last RX, interrupts are
 * disabled, status is set to IDLE, and the registered callback is fired.
 * ========================================================================= */
void SPI1_IRQHandler(void) {

    /* Snapshot SR once to avoid re-reading a changing register */
    uint32 sr = SPI1->SR;

    /* ------------------------------------------------------------------
     * 1. TX path: transmit buffer empty — load next byte
     * ------------------------------------------------------------------ */
    if ((SPI1->CR2 & (1U << SPI_CR2_TXEIE_POS)) &&
        (sr         & (1U << SPI_SR_TXE_POS)))
    {
        if (spi_Index < SPI_FRAME_SIZE) {
            /* Cast away volatile for the DR write — the write itself is
             * the intended side-effect, no caching issue here. */
            SPI1->DR = (uint32)spi_TxBuf[spi_Index];
            spi_Index++;
        } else {
            /* All bytes queued.  Disable TXEIE to stop spurious re-entry.
             * RXNEIE remains active until the last byte is clocked in. */
            SPI1->CR2 &= ~(1U << SPI_CR2_TXEIE_POS);
        }
    }

    /* ------------------------------------------------------------------
     * 2. RX path: received byte ready — store it
     * ------------------------------------------------------------------ */
    if (sr & (1U << SPI_SR_RXNE_POS)) {

        /* Reading DR clears the RXNE flag */
        uint8 rxByte = (uint8)SPI1->DR;

        /* spi_Index already points one ahead of the byte just transmitted,
         * so the received byte maps to slot (spi_Index - 1).
         * Boundary guard prevents an overrun if the ISR fires unexpectedly. */
        uint8 rxSlot = spi_Index - 1U;
        if (rxSlot < SPI_FRAME_SIZE) {
            spi_RxBuf[rxSlot] = rxByte;
        }

        /* Check for full-frame completion */
        if (spi_Index >= SPI_FRAME_SIZE) {
            prv_DisableDataIrqs();
            spi_Status = SPI_STATUS_IDLE;

            /* Notify IPC Manager — runs in ISR context, must be brief */
            if (spi_Callback != (void *)0) {
                spi_Callback((uint8 *)spi_RxBuf);
            }
        }
    }
}