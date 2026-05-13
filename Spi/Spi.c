#include "Gpio.h"
#include "Spi.h"
#define SPI_BR_DIVIDER                0x7U   /* fPCLK / 256 = 62.5 kHz (stable for simulation) */

/* =========================================================================
 * Module-private state
 * ========================================================================= */
static volatile uint8           *spi_TxBuf    = (void *)0;
static volatile uint8           *spi_RxBuf    = (void *)0;
static volatile uint8            spi_TxIdx    = 0U;
static volatile uint8            spi_RxIdx    = 0U;
static volatile Spi_StatusType   spi_Status   = SPI_STATUS_IDLE;
static          Spi_FrameCallback spi_Callback = (void *)0;

/* Track role so StartTransfer knows whether to pre-load */
static uint8 spi_Role = SPI_MASTER;

/* =========================================================================
 * Private helpers
 * ========================================================================= */
static void prv_EnableDataIrqs(void) {
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

    spi_Role = MasterSlave;

    /* ----- Data GPIO: PB3=SCK, PB4=MISO, PB5=MOSI (AF5) ----------------- */
    Gpio_Init(GPIO_B, 3U, GPIO_AF, GPIO_PUSH_PULL);
    Gpio_Init(GPIO_B, 4U, GPIO_AF, GPIO_PUSH_PULL);
    Gpio_Init(GPIO_B, 5U, GPIO_AF, GPIO_PUSH_PULL);
    Gpio_SetAF(GPIO_B, 3U, GPIO_AF5);
    Gpio_SetAF(GPIO_B, 4U, GPIO_AF5);
    Gpio_SetAF(GPIO_B, 5U, GPIO_AF5);

    /* ----- NSS / CS setup ------------------------------------------------
     * MASTER: PA4 = GPIO push-pull output, idle HIGH (software CS).
     *         SSM=1, SSI=1.
     * SLAVE : PA4 = SPI1_NSS, AF5  (PA4 is SPI1_NSS on STM32F401).
     *         SSM=0 — hardware NSS gates the MISO output driver.
     * --------------------------------------------------------------------- */
    if (MasterSlave == SPI_MASTER) {
        Gpio_Init(GPIO_A, SPI_CS_PIN, GPIO_OUTPUT, GPIO_PUSH_PULL);
        Gpio_WritePin(GPIO_A, SPI_CS_PIN, 1U);
    } else {
        Gpio_Init(GPIO_A, 4U, GPIO_AF, GPIO_PUSH_PULL);
        Gpio_SetAF(GPIO_A, 4U, GPIO_AF5);
    }

    /* ----- Disable SPI --------------------------------------------------- */
    SPI1->CR1 &= ~(1U << SPI_CR1_SPE_POS);

    /* ----- SSM / SSI ----------------------------------------------------- */
    if (MasterSlave == SPI_MASTER) {
        SPI1->CR1 |=  (1U << SPI_CR1_SSM_POS);
        SPI1->CR1 |=  (1U << SPI_CR1_SSI_POS);
    } else {
        SPI1->CR1 &= ~(1U << SPI_CR1_SSM_POS);
        SPI1->CR1 &= ~(1U << SPI_CR1_SSI_POS);
    }

    /* ----- Master / Slave ------------------------------------------------ */
    SPI1->CR1 &= ~(1U << SPI_CR1_MSTR_POS);
    SPI1->CR1 |=  ((uint32)MasterSlave << SPI_CR1_MSTR_POS);

    /* ----- Clock polarity / phase ---------------------------------------- */
    SPI1->CR1 &= ~(1U << SPI_CR1_CPOL_POS);
    SPI1->CR1 |=  ((uint32)ClkPol   << SPI_CR1_CPOL_POS);
    SPI1->CR1 &= ~(1U << SPI_CR1_CPHA_POS);
    SPI1->CR1 |=  ((uint32)ClkPhase << SPI_CR1_CPHA_POS);

    /* ----- Baud rate ----------------------------------------------------- */
    SPI1->CR1 &= ~SPI_CR1_BR_MASK;
    SPI1->CR1 |=  ((uint32)SPI_BR_DIVIDER << SPI_CR1_BR_POS);

    /* ----- 8-bit frame --------------------------------------------------- */
    SPI1->CR1 &= ~SPI_CR1_DFF_MASK;

    /* ----- Enable SPI ---------------------------------------------------- */
    SPI1->CR1 |= (1U << SPI_CR1_SPE_POS);

    /* ----- Reset state --------------------------------------------------- */
    spi_TxBuf    = (void *)0;
    spi_RxBuf    = (void *)0;
    spi_TxIdx    = 0U;
    spi_RxIdx    = 0U;
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
 * Spi1_AssertCS / Spi1_DeassertCS  (master only)
 * ========================================================================= */
void Spi1_AssertCS(void) {
    Gpio_WritePin(GPIO_A, SPI_CS_PIN, 0U);
}

void Spi1_DeassertCS(void) {
    Gpio_WritePin(GPIO_A, SPI_CS_PIN, 1U);
}

/* =========================================================================
 * Spi1_StartTransfer
 * ========================================================================= */
uint8 Spi1_StartTransfer(uint8 *TxBuffer, uint8 *RxBuffer) {

    if (spi_Status == SPI_STATUS_BUSY) {
        return SPI_NOK;
    }

    /* Clear any stale error flags (OVR) before starting */
    (void)SPI1->DR;
    (void)SPI1->SR;

    spi_TxBuf  = TxBuffer;
    spi_RxBuf  = RxBuffer;
    spi_TxIdx  = 0U;
    spi_RxIdx  = 0U;
    spi_Status = SPI_STATUS_BUSY;

    /* Pre-load byte 0 into the hardware shift register.
     * MASTER: pre-loads immediately to clock data out on SCK rising edge.
     * SLAVE:  does NOT pre-load to avoid one-byte frame shift; 
     *         waits for Master's clock to synchronize first byte TX/RX. */
    if (SPI1->SR & (1U << SPI_SR_TXE_POS)) {
        SPI1->DR  = (uint32)spi_TxBuf[0];
        spi_TxIdx = 1U;
    }

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
 * ========================================================================= */
void SPI1_IRQHandler(void) {

    uint32 sr = SPI1->SR;

    /* A. Error handling (OVR)
     * Clear OVR by reading DR then reading SR. */
    if (sr & (1U << 6)) {
        volatile uint32 dummy;
        dummy = SPI1->DR;
        dummy = SPI1->SR;
        (void)dummy;
    }

    /* B. RX Path */
    if (sr & (1U << SPI_SR_RXNE_POS)) {
        uint8 rxByte = (uint8)SPI1->DR;
        if (spi_RxIdx < SPI_FRAME_SIZE) {
            spi_RxBuf[spi_RxIdx] = rxByte;
            spi_RxIdx++;
        }

        if (spi_RxIdx >= SPI_FRAME_SIZE) {
            prv_DisableDataIrqs();
            spi_Status = SPI_STATUS_IDLE;
            if (spi_Callback != (void *)0) {
                spi_Callback((uint8 *)spi_RxBuf);
            }
        }
    }

    /* C. TX Path */
    if ((SPI1->CR2 & (1U << SPI_CR2_TXEIE_POS)) && (SPI1->SR & (1U << SPI_SR_TXE_POS)))
    {
        if (spi_TxIdx < SPI_FRAME_SIZE) {
            SPI1->DR = (uint32)spi_TxBuf[spi_TxIdx];
            spi_TxIdx++;
        } else {
            SPI1->CR2 &= ~(1U << SPI_CR2_TXEIE_POS);
        }
    }
}