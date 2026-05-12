#include "Rcc.h"

#include "Bit_Operations.h"
#include "Rcc_Private.h"
#include "Std_Types.h"

void Rcc_Init(void) {
    /* Enable HSI (Internal 16 MHz RC oscillator) */
    SET_BIT(RCC_CR, 0);
    
    /* Wait until HSI is ready (HSIRDY bit is set by hardware) */
    while (READ_BIT(RCC_CR, 1) == 0) {
        /* Wait for clock stability */
    }
}

void Rcc_Enable(uint8 PeripheralId) {
    uint8 BusId = PeripheralId / 32;
    uint8 PeripheralBitPosition = PeripheralId % 32;

    switch (BusId) {
        case RCC_AHB1:
            SET_BIT(RCC_AHB1ENR, PeripheralBitPosition);
            break;
        case RCC_AHB2:
            SET_BIT(RCC_AHB2ENR, PeripheralBitPosition);
            break;
        case RCC_APB1:
            SET_BIT(RCC_APB1ENR, PeripheralBitPosition);
            break;
        case RCC_APB2:
            SET_BIT(RCC_APB2ENR, PeripheralBitPosition);
            break;
        default:
            break;
    }
}

void Rcc_Disable(uint8 PeripheralId) {
    uint8 BusId = PeripheralId / 32;
    uint8 PeripheralBitPosition = PeripheralId % 32;
    switch (BusId) {
        case RCC_AHB1:
            CLEAR_BIT(RCC_AHB1ENR, PeripheralBitPosition);
            break;
        case RCC_AHB2:
            CLEAR_BIT(RCC_AHB2ENR, PeripheralBitPosition);
            break;
        case RCC_APB1:
            CLEAR_BIT(RCC_APB1ENR, PeripheralBitPosition);
            break;
        case RCC_APB2:
            CLEAR_BIT(RCC_APB2ENR, PeripheralBitPosition);
            break;
        default:
            break;
    }
}

