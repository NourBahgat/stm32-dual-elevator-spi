#ifndef UTILS_H
#define UTILS_H

#include "Std_Types.h"
#define REG32(ADDR)  ( *(volatile uint32 *)(ADDR) )

/* Critical Section Macros for ARM Cortex-M */
#define Enter_Critical()  __asm volatile ("cpsid i" : : : "memory")
#define Exit_Critical()   __asm volatile ("cpsie i" : : : "memory")

#endif /* UTILS_H */
