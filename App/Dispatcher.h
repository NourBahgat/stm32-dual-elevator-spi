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
 *    P1 – Immediate   : elevator already at call_floor AND IDLE
 *    P2 – Perfect Match : elevator moving toward call_floor, same direction
 *    P3 – Passed Match  : same direction but already past the floor
 *    P4 – Nearest IDLE  : no directional match; closest idle elevator
 *    P5 – Opposite Dir  : both going wrong way → defer (DISPATCH_NONE)
 *    P6 – Absolute fallback: closest available (e.g. DOORS_OPEN)
 *
 * @param  call_floor      Floor of the hall-call button press (1-4)
 * @param  call_direction  DIR_UP or DIR_DOWN
 * @return DISPATCH_A, DISPATCH_B, or DISPATCH_NONE
 */
Dispatch_Result Dispatcher_AssignElevator(uint8 call_floor, uint8 call_direction);

/**
 * @brief  Periodic 50 ms tick — retries any deferred call once an elevator recovers.
 */
void Dispatcher_Tick(void);

/*
 * NOTE: Dispatcher_CheckCommFault() has been removed.
 * Comm-fault detection and recovery are handled entirely by IPC_MasterTick()
 * and IPC_OnFrameReceived() inside IPC.c.  The Dispatcher reads the shared
 * Ipc_CommFault flag directly; no additional function is needed here.
 */

#endif /* DISPATCHER_H */
