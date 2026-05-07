#ifndef GPIO_H
#define GPIO_H

#include "Std_Types.h"


/*PortName*/
#define GPIO_A     'A'
#define GPIO_B     'B'
#define GPIO_C     'C'
#define GPIO_D     'D'

/*PinMode*/
#define GPIO_INPUT  0x00
#define GPIO_OUTPUT 0x01
#define GPIO_AF     0x02
#define GPIO_ANALOG 0x03

/*AlternateFunction numbers (passed to Gpio_SetAF)*/
#define GPIO_AF0    0U   /* SYS / MCO / JTAG / SWD          */
#define GPIO_AF1    1U   /* TIM1 / TIM2                      */
#define GPIO_AF2    2U   /* TIM3 / TIM4 / TIM5               */
#define GPIO_AF3    3U   /* TIM8 / TIM9 / TIM10 / TIM11      */
#define GPIO_AF4    4U   /* I2C1 / I2C2 / I2C3               */
#define GPIO_AF5    5U   /* SPI1 / SPI2 / SPI3 / SPI4        */
#define GPIO_AF6    6U   /* SPI2 / SPI3 / SAI1               */
#define GPIO_AF7    7U   /* USART1 / USART2 / USART3         */
#define GPIO_AF8    8U   /* USART4 / USART5 / USART6         */
#define GPIO_AF9    9U   /* CAN1 / CAN2 / TIM12 / TIM13/14  */
#define GPIO_AF10   10U  /* OTG_FS / OTG_HS                  */
#define GPIO_AF11   11U  /* ETH                              */
#define GPIO_AF12   12U  /* FMC / SDIO / OTG_HS              */
#define GPIO_AF13   13U  /* DCMI                             */
#define GPIO_AF14   14U  /* reserved                         */
#define GPIO_AF15   15U  /* EVENTOUT                         */

/*DefaultState*/
#define GPIO_PUSH_PULL  0x00
#define GPIO_OPEN_DRAIN 0x01

#define GPIO_NO_PULL_DOWN 0x00
#define GPIO_PULL_UP      0x01
#define GPIO_PULL_DOWN    0x02

/*Data*/
#define LOW      0x0
#define HIGH     0x1


#define OK  0x0
#define NOK 0x1


void Gpio_Init(uint8 PortName, uint8 PinNumber, uint8 PinMode, uint8 DefaultState);

/*
 * Gpio_SetAF
 * ----------
 * Writes the alternate-function number (0-15) for a pin into the
 * appropriate AFR register (AFRL for pins 0-7, AFRH for pins 8-15).
 *
 * Must be called AFTER Gpio_Init with PinMode = GPIO_AF.
 * Has no effect if the pin is not in AF mode.
 *
 * Example — PB4 as TIM3_CH1 (AF2):
 *   Gpio_Init(GPIO_B, 4, GPIO_AF, GPIO_PUSH_PULL);
 *   Gpio_SetAF(GPIO_B, 4, GPIO_AF2);
 */
void Gpio_SetAF(uint8 PortName, uint8 PinNumber, uint8 AfNumber);

uint8 Gpio_WritePin(uint8 PortName, uint8 PinNumber, uint8 Data);

uint8 Gpio_ReadPin(uint8 PortName, uint8 PinNumber);

#endif //GPIO_H
