#ifndef SPI_H
#define SPI_H

#include "Std_Types.h"

/* =========================================================================
 * SPI1 Register Map  (STM32F401 Reference Manual, Section 20.5)
 * Base address: 0x40013000
 * All registers are 32-bit; only the lower 16 bits are implemented by hw.
 * ========================================================================= */
typedef struct {
    volatile uint32 CR1;        /* 0x00  Control register 1          */
    volatile uint32 CR2;        /* 0x04  Control register 2          */
    volatile uint32 SR;         /* 0x08  Status register             */
    volatile uint32 DR;         /* 0x0C  Data register               */
    volatile uint32 CRCPR;      /* 0x10  CRC polynomial register     */
    volatile uint32 RXCRCR;     /* 0x14  RX CRC register             */
    volatile uint32 TXCRCR;     /* 0x18  TX CRC register             */
    volatile uint32 I2SCFGR;    /* 0x1C  I2S configuration register  */
    volatile uint32 I2SPR;      /* 0x20  I2S prescaler register      */
} SpiType;

#define SPI1_BASE_ADDR   0x40013000UL
#define SPI1             ((SpiType *)SPI1_BASE_ADDR)

/* =========================================================================
 * CR1 Bit Positions
 * ========================================================================= */
#define SPI_CR1_CPHA_POS     0U   /* Clock phase                        */
#define SPI_CR1_CPOL_POS     1U   /* Clock polarity                     */
#define SPI_CR1_MSTR_POS     2U   /* Master selection                   */
#define SPI_CR1_BR_POS       3U   /* Baud rate control [5:3]            */
#define SPI_CR1_SPE_POS      6U   /* SPI enable                         */
#define SPI_CR1_SSI_POS      8U   /* Internal slave select              */
#define SPI_CR1_SSM_POS      9U   /* Software slave management          */
#define SPI_CR1_DFF_POS      11U  /* Data frame format (0=8bit,1=16bit) */

/* CR1 Masks */
#define SPI_CR1_BR_MASK      (0x7U << SPI_CR1_BR_POS)
#define SPI_CR1_DFF_MASK     (0x1U << SPI_CR1_DFF_POS)

/* =========================================================================
 * CR2 Bit Positions
 * ========================================================================= */
#define SPI_CR2_RXNEIE_POS   6U   /* RX buffer not-empty interrupt enable */
#define SPI_CR2_TXEIE_POS    7U   /* TX buffer empty interrupt enable     */

/* =========================================================================
 * SR Bit Positions
 * ========================================================================= */
#define SPI_SR_RXNE_POS      0U   /* Receive buffer not empty */
#define SPI_SR_TXE_POS       1U   /* Transmit buffer empty    */
#define SPI_SR_BSY_POS       7U   /* Busy flag                */

/* =========================================================================
 * SPI Frame Definition
 * ========================================================================= */
#define SPI_FRAME_SIZE       8U

/*
 * Frame Layout (8 bytes):
 *  [0] Header    = 0xA5   (sync / start-of-frame marker)
 *  [1] FSM State          (IDLE / MOVING_UP / MOVING_DOWN / DOORS_OPEN / EMERGENCY)
 *  [2] Floor              (1 - 4)
 *  [3] Direction          (NONE=0 / UP=1 / DOWN=2)
 *  [4] Queue_Hi           (pending floor-request bitmask, high byte)
 *  [5] Queue_Lo           (pending floor-request bitmask, low byte)
 *  [6] Flags              (bit-packed: bit0=door_sensor, bit1=comm_fault_ack)
 *  [7] Checksum           (XOR of bytes 0-6)
 */
#define SPI_HEADER_BYTE      0xA5U

/* Frame byte indices */
#define SPI_IDX_HEADER       0U
#define SPI_IDX_STATE        1U
#define SPI_IDX_FLOOR        2U
#define SPI_IDX_DIRECTION    3U
#define SPI_IDX_QUEUE_HI     4U
#define SPI_IDX_QUEUE_LO     5U
#define SPI_IDX_FLAGS        6U
#define SPI_IDX_CHECKSUM     7U

/* =========================================================================
 * Driver Configuration Macros
 * ========================================================================= */

/* Master or Slave */
#define SPI_SLAVE                     0U
#define SPI_MASTER                    1U

/* Clock Polarity */
#define SPI_IDLE_LOW                  0U
#define SPI_IDLE_HIGH                 1U

/* Clock Phase */
#define SPI_SAMPLE_FIRST_TRANSITION   0U   /* Capture on first  edge */
#define SPI_SAMPLE_SECOND_TRANSITION  1U   /* Capture on second edge */

/* Baud rate: BR[2:0] = 001 -> fPCLK/4
 * APB2 on STM32F401 = 84 MHz -> SPI clock = 21 MHz.
 * Increase value to slow down if signal integrity requires it. */
#define SPI_BR_DIVIDER                0x1U

/* Return codes */
#define SPI_OK                        0U
#define SPI_NOK                       1U

/* =========================================================================
 * Transfer Status
 * ========================================================================= */
typedef enum {
    SPI_STATUS_IDLE = 0U,   /* Transfer finished; RX buffer is valid    */
    SPI_STATUS_BUSY         /* Transfer in progress; RX buffer is stale */
} Spi_StatusType;

/* =========================================================================
 * Callback Type
 * Registered by the IPC Manager.
 * Invoked from ISR context when a complete SPI_FRAME_SIZE frame arrives.
 * Keep the implementation short — no blocking calls inside.
 * ========================================================================= */
typedef void (*Spi_FrameCallback)(uint8 *RxBuffer);

/* =========================================================================
 * Public API
 * ========================================================================= */

/**
 * @brief  Initialise SPI1 peripheral and its GPIO alternate functions.
 *         GPIO: PB3=SCK, PB4=MISO, PB5=MOSI (AF5).
 *         Does NOT enable TX/RX interrupts — call Spi1_StartTransfer() for that.
 *
 * @param  MasterSlave  SPI_MASTER or SPI_SLAVE
 * @param  ClkPol       SPI_IDLE_LOW or SPI_IDLE_HIGH
 * @param  ClkPhase     SPI_SAMPLE_FIRST_TRANSITION or SPI_SAMPLE_SECOND_TRANSITION
 */
void Spi1_Init(uint8 MasterSlave, uint8 ClkPol, uint8 ClkPhase);

/**
 * @brief  Register a callback invoked from the ISR on full-frame reception.
 *         Call once after Spi1_Init, before the first Spi1_StartTransfer.
 *
 * @param  Callback  Function pointer; pass NULL to disable.
 */
void Spi1_RegisterCallback(Spi_FrameCallback Callback);

/**
 * @brief  Arm the interrupt-driven engine for one SPI_FRAME_SIZE exchange.
 *
 *         MASTER - call from the 50 ms timer ISR each cycle:
 *             IPC_PackFrame(masterTxBuf);
 *             Spi1_StartTransfer(masterTxBuf, masterRxBuf);
 *
 *         SLAVE  - call once after init and again inside the RX callback
 *                  to re-arm for the next master cycle:
 *             IPC_PackFrame(slaveTxBuf);
 *             Spi1_StartTransfer(slaveTxBuf, slaveRxBuf);
 *
 *         TxBuffer and RxBuffer MUST be static or global (not stack variables).
 *         They must remain valid until status returns SPI_STATUS_IDLE.
 *
 * @param  TxBuffer  Pointer to SPI_FRAME_SIZE bytes to transmit.
 * @param  RxBuffer  Pointer to SPI_FRAME_SIZE byte receive buffer.
 * @return SPI_OK    Driver armed successfully.
 *         SPI_NOK   Previous transfer still running; caller should retry.
 */
uint8 Spi1_StartTransfer(uint8 *TxBuffer, uint8 *RxBuffer);

/**
 * @brief  Poll the driver state from main-loop / FSM context.
 * @return SPI_STATUS_IDLE  Last frame fully received; RxBuffer is valid.
 *         SPI_STATUS_BUSY  Transfer in progress.
 */
Spi_StatusType Spi1_GetStatus(void);

/**
 * @brief  SPI1 global interrupt handler.
 *         Wire to the vector table as SPI1_IRQHandler (startup_stm32f401xe.s).
 *         Handles TXEIE and RXNEIE flags in one ISR entry.
 */
void SPI1_IRQHandler(void);

#endif /* SPI_H */