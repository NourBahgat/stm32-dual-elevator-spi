#ifndef NVIC_H
#define NVIC_H

#include "Std_Types.h"

void Nvic_EnableIrq(uint8 IrqNum);

void Nvic_DisableIrq(uint8 IrqNum);

void Nvic_SetPriority(uint8 IrqNum, uint8 Priority);

#endif /* NVIC_H */
