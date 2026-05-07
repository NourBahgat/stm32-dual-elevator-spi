#include "Nvic.h"
#include "Nvic_Private.h"
#include "Bit_Operations.h"

/*
 * Each ISER/ICER register covers 32 IRQ lines.
 *   Register index  = IrqNum / 32
 *   Bit position    = IrqNum % 32
 *
 * STM32F401 priority field width = 4 bits (bits [7:4] of each IPR byte).
 * Writing priority: IPR byte for IrqNum = IPR[IrqNum/4], bits [(IrqNum%4)*8 + 7 : (IrqNum%4)*8 + 4]
 */

void Nvic_EnableIrq(uint8 IrqNum) {
    uint8 regIndex = IrqNum / 32U;
    uint8 bitPos   = IrqNum % 32U;
    SET_BIT(NVIC_REG(NVIC_ISER_BASE, regIndex), bitPos);
}

void Nvic_DisableIrq(uint8 IrqNum) {
    uint8 regIndex = IrqNum / 32U;
    uint8 bitPos   = IrqNum % 32U;
    /* Writing 1 to ICER clears the enable; writing 0 has no effect */
    SET_BIT(NVIC_REG(NVIC_ICER_BASE, regIndex), bitPos);
}

void Nvic_SetPriority(uint8 IrqNum, uint8 Priority) {
    /* Each IPR register holds 4 priority bytes.
     * STM32F401 implements bits [7:4]; shift priority into the upper nibble. */
    uint8  regIndex  = IrqNum / 4U;
    uint8  byteShift = (IrqNum % 4U) * 8U;
    uint32 mask      = (uint32)0xFFU << byteShift;
    uint32 value     = (uint32)(Priority << 4U) << byteShift;

    NVIC_REG(NVIC_IPR_BASE, regIndex) &= ~mask;
    NVIC_REG(NVIC_IPR_BASE, regIndex) |= value;
}
