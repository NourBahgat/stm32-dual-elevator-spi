#ifndef SPI_H
#define SPI_H

#include "Std_Types.h"

/* =========================================================================
 * SPI1 Register Map  (STM32F401 Reference Manual, Section 20.5)
 * Base address: 0x40013000
 * ========================================================================= */
typedef struct {
    volatile uint32 CR1;
    volatile uint32 CR2;
    volatile uint32 SR;
    volatile uint32 DR;
    volatile uint32 CRCPR;
    volatile uint32 RXCRCR;
    volatile uint32 TXCRCR;
    volatile uint32 I2SCFGR;
    volatile uint32 I2SPR;
} SpiType;

#define SPI1_BASE_ADDR   0x40013000UL
#define SPI1             ((SpiType *)SPI1_BASE_ADDR)

/* =========================================================================
 * CR1 Bit Positions
 * ========================================================================= */
#define SPI_CR1_CPHA_POS     0U
#define SPI_CR1_CPOL_POS     1U
#define SPI_CR1_MSTR_POS     2U
#define SPI_CR1_BR_POS       3U
#define SPI_CR1_SPE_POS      6U
#define SPI_CR1_SSI_POS      8U
#define SPI_CR1_SSM_POS      9U
#define SPI_CR1_DFF_POS      11U

#define SPI_CR1_BR_MASK      (0x7U << SPI_CR1_BR_POS)
#define SPI_CR1_DFF_MASK     (0x1U << SPI_CR1_DFF_POS)

/* =========================================================================
 * CR2 Bit Positions
 * ========================================================================= */
#define SPI_CR2_RXNEIE_POS   6U
#define SPI_CR2_TXEIE_POS    7U

/* =========================================================================
 * SR Bit Positions
 * ========================================================================= */
#define SPI_SR_RXNE_POS      0U
#define SPI_SR_TXE_POS       1U
#define SPI_SR_BSY_POS       7U

/* =========================================================================
 * SPI Frame Definition
 * ========================================================================= */
#define SPI_FRAME_SIZE       8U

#define SPI_HEADER_BYTE      0xA5U

#define SPI_IDX_HEADER       0U
#define SPI_IDX_STATE        1U
#define SPI_IDX_FLOOR        2U
#define SPI_IDX_DIRECTION    3U
#define SPI_IDX_QUEUE_HI     4U
#define SPI_IDX_QUEUE_LO     5U
#define SPI_IDX_FLAGS        6U
#define SPI_IDX_CHECKSUM     7U

/* =========================================================================
 * CS / NSS Pin Configuration
 *
 * SPI1 AF5 pin mapping on STM32F401:
 *   SCK  = PB3
 *   MISO = PB4
 *   MOSI = PB5
 *   NSS  = PA4  ← SPI1_NSS (PB12 = SPI2_NSS, do NOT use for SPI1)
 *
 * MASTER: PA4 = GPIO push-pull output (software CS). SSM=1, SSI=1.
 * SLAVE : PA4 = AF5 hardware NSS input.              SSM=0.
 *
 * Proteus wire: Master PA4 → Slave PA4.
 * ========================================================================= */
#define SPI_CS_PIN           4U   /* PA4 */

/* =========================================================================
 * Driver Configuration
 * ========================================================================= */
#define SPI_SLAVE                     0U
#define SPI_MASTER                    1U

#define SPI_IDLE_LOW                  0U
#define SPI_IDLE_HIGH                 1U

#define SPI_SAMPLE_FIRST_TRANSITION   0U
#define SPI_SAMPLE_SECOND_TRANSITION  1U

#define SPI_BR_DIVIDER                0x1U   /* fPCLK/4 */

#define SPI_OK                        0U
#define SPI_NOK                       1U

/* =========================================================================
 * Transfer Status
 * ========================================================================= */
typedef enum {
    SPI_STATUS_IDLE = 0U,
    SPI_STATUS_BUSY
} Spi_StatusType;

/* =========================================================================
 * Callback
 * ========================================================================= */
typedef void (*Spi_FrameCallback)(uint8 *RxBuffer);

/* =========================================================================
 * Public API
 * ========================================================================= */

/**
 * @brief  Initialise SPI1 peripheral and GPIO.
 *
 *         Data pins: PB3=SCK, PB4=MISO, PB5=MOSI (AF5, both roles).
 *         CS/NSS:    PA4 = GPIO output (master) or AF5 NSS input (slave).
 *
 *         Internally records MasterSlave role so Spi1_StartTransfer can
 *         decide whether to pre-load DR (master only).
 */
void Spi1_Init(uint8 MasterSlave, uint8 ClkPol, uint8 ClkPhase);

/**
 * @brief  Register callback invoked from ISR after full frame received.
 */
void Spi1_RegisterCallback(Spi_FrameCallback Callback);

/**
 * @brief  Drive PA4 LOW — selects slave.  Master only.
 *         Must be called BEFORE Spi1_StartTransfer().
 */
void Spi1_AssertCS(void);

/**
 * @brief  Drive PA4 HIGH — releases slave.  Master only.
 *         Call from inside the OnFrameRx callback.
 */
void Spi1_DeassertCS(void);

/**
 * @brief  Arm interrupt engine for one SPI_FRAME_SIZE full-duplex exchange.
 *
 *         MASTER: pre-loads byte 0 into DR (SCK starts immediately).
 *                 AssertCS() must already be LOW before calling this.
 *
 *         SLAVE:  does NOT pre-load byte 0 — avoids the one-byte frame
 *                 shift that occurs when DR is written before NSS goes LOW.
 *                 TXEIE fires instantly on enable and the ISR writes byte 0
 *                 well before the master clocks the first SCK edge.
 *
 * @param  TxBuffer  Static/global SPI_FRAME_SIZE byte transmit buffer.
 * @param  RxBuffer  Static/global SPI_FRAME_SIZE byte receive buffer.
 * @return SPI_OK or SPI_NOK (busy).
 */
uint8 Spi1_StartTransfer(uint8 *TxBuffer, uint8 *RxBuffer);

/**
 * @brief  Poll transfer status.
 */
Spi_StatusType Spi1_GetStatus(void);

/**
 * @brief  SPI1 interrupt handler — wire to vector table as SPI1_IRQHandler.
 */
void SPI1_IRQHandler(void);

#endif /* SPI_H */