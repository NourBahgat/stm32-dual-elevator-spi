#ifndef NVIC_PRIVATE_H
#define NVIC_PRIVATE_H

#include "Std_Types.h"
#include "Utils.h"

/*
 * NVIC register base addresses (ARM Cortex-M4, PM0214 §4.3)
 *
 *  ISER[0..7]  0xE000E100 – Interrupt Set-Enable    (8 x 32-bit, covers IRQ 0-239)
 *  ICER[0..7]  0xE000E180 – Interrupt Clear-Enable
 *  ISPR[0..7]  0xE000E200 – Interrupt Set-Pending
 *  ICPR[0..7]  0xE000E280 – Interrupt Clear-Pending
 *  IPR[0..59]  0xE000E400 – Interrupt Priority       (8-bit fields, 4 per register)
 */
#define NVIC_ISER_BASE   0xE000E100UL
#define NVIC_ICER_BASE   0xE000E180UL
#define NVIC_ISPR_BASE   0xE000E200UL
#define NVIC_ICPR_BASE   0xE000E280UL
#define NVIC_IPR_BASE    0xE000E400UL

/* Access register n in an array starting at base */
#define NVIC_REG(BASE, N)   REG32((BASE) + ((N) * 4UL))

#endif /* NVIC_PRIVATE_H */
