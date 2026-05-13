#ifndef IWDG_H
#define IWDG_H

#include "Std_Types.h"

/**
 * @brief  Initializes the Independent Watchdog (IWDG).
 *         Once started, the IWDG cannot be disabled except by a reset.
 * 
 * @param  timeout_ms: Requested timeout in milliseconds (max ~32700ms).
 */
void Iwdg_Init(uint32 timeout_ms);

/**
 * @brief  Reloads the IWDG counter with the value in the RLR register.
 *         This must be called periodically to prevent a system reset.
 */
void Iwdg_Refresh(void);

#endif /* IWDG_H */
