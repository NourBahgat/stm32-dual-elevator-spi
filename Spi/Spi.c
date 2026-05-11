/**
 * Spi.c
 *
 *  Created on: Wed May 24 2023
 *  Author    : Abdullah Darwish
 */
#include "Spi.h"
#include "Gpio.h"
#include "stm32f401xe.h"

void Spi1_Init(uint8 MasterSlave, uint8 ClkPol, uint8 ClkPhase) {

  Gpio_Init(GPIO_B, 3, GPIO_AF, GPIO_PUSH_PULL);
  Gpio_Init(GPIO_B, 4, GPIO_AF, GPIO_PUSH_PULL);
  Gpio_Init(GPIO_B, 5, GPIO_AF, GPIO_PUSH_PULL);

  Gpio_SetAF(GPIO_B, 3, GPIO_AF5);
  Gpio_SetAF(GPIO_B, 4, GPIO_AF5);
  Gpio_SetAF(GPIO_B, 5, GPIO_AF5);

  SPI1->CR1 |= (0x1 << SPI_CR1_SSM_Pos);
  SPI1->CR1 |= (0x1 << SPI_CR1_SSI_Pos);

  SPI1->CR1 &= ~(1 << SPI_CR1_MSTR_Pos);
  SPI1->CR1 |= (MasterSlave << SPI_CR1_MSTR_Pos);

  SPI1->CR1 &= ~(1 << SPI_CR1_CPOL_Pos);
  SPI1->CR1 |= (ClkPol << SPI_CR1_CPOL_Pos);

  SPI1->CR1 &= ~(1 << SPI_CR1_CPHA_Pos);
  SPI1->CR1 |= (ClkPhase << SPI_CR1_CPHA_Pos);

  /*************************************************************************/
  // Baud Rate
  SPI1->CR1 &= ~(0x7 << SPI_CR1_BR_Pos);
  SPI1->CR1 |= (0x1 << SPI_CR1_BR_Pos); // 16/4 -> 4MHZ
  /*************************************************************************/

  SPI1->CR1 |= (1 << SPI_CR1_SPE_Pos);
}

uint8 Spi1_TransmitReceiveByte(uint8 TxData, uint8 *RxData) {
    /* Wait for TXE (TX buffer empty) */
    while (!(SPI1->SR & (1 << SPI_SR_TXE_Pos)));
    SPI1->DR = TxData;
    /* Wait for RXNE (RX buffer not empty) */
    while (!(SPI1->SR & (1 << SPI_SR_RXNE_Pos)));
    *RxData = (uint8)SPI1->DR;
    return SPI_OK;
}

/* ── SPI callback support ─────────────────────────────────────────────── */

static SpiCallback spi1_UserCallback = 0;

void Spi1_RegisterCallback(SpiCallback Callback)
{
    spi1_UserCallback = Callback;
}

/*
 * Spi1_StartTransfer
 * Performs a synchronous full-duplex SPI frame transfer (SPI_FRAME_SIZE
 * bytes), then invokes the registered callback with the received buffer.
 *
 * Called from IPC layer — IPC_Init() pre-arms the slave, and
 * IPC_OnFrameReceived() re-arms after each received frame.
 */
void Spi1_StartTransfer(uint8 *TxBuf, uint8 *RxBuf)
{
    if (TxBuf == 0 || RxBuf == 0) return;

    for (uint8 i = 0U; i < 8U; i++) {
        (void)Spi1_TransmitReceiveByte(TxBuf[i], &RxBuf[i]);
    }

    if (spi1_UserCallback != 0) {
        spi1_UserCallback(RxBuf);
    }
}
