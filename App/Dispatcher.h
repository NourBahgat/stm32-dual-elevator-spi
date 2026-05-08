#ifndef DISPATCHER_H
#define DISPATCHER_H

#include "Std_Types.h"
#include "Ipc.h"
#include "Elevator_FSM.h"

/* ── Which elevator to dispatch ───────────────────────────────────── */
typedef enum {
    DISPATCH_A    = 0U,   /* Master's own elevator  (Elevator A) */
    DISPATCH_B    = 1U,   /* Remote slave elevator  (Elevator B) */
    DISPATCH_NONE = 0xFFU /* Both in FAULT — call deferred  */
} Dispatch_Result;

/* ── Public API ───────────────────────────────────────────────────── */

/**
 * @brief  Initialise the Dispatcher module.
 *         Call after IPC_Init() so the remote state buffer is ready.
 */
void Dispatcher_Init(void);

/**
 * @brief  Assign the best elevator to a hall call.
 *
 *  Priority (highest → lowest):
 *    P1 – elevator already at call_floor AND IDLE
 *    P2 – elevator moving toward call_floor in the same direction
 *    P3 – Nearest IDLE elevator
 *    P4 – Fallback: closest elevator (both going opposite direction)
 *
 * @param  call_floor      Floor of the hall-call button press (1-4)
 * @param  call_direction  DIR_UP or DIR_DOWN
 * @return DISPATCH_A, DISPATCH_B, or DISPATCH_NONE
 */
Dispatch_Result Dispatcher_AssignElevator(uint8 call_floor, uint8 call_direction);

/**
 * @brief  Periodic 50 ms tick — retries any deferred call once a elevator recovers.
 */
void Dispatcher_Tick(void);

#endif /* DISPATCHER_H */
