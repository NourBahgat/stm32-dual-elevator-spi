#ifndef ELEVATOR_FSM_H
#define ELEVATOR_FSM_H

#include "IPC.h"
#include "Std_Types.h"


/* ── Floor range ──────────────────────────────────────────────────── */
#define ELEVATOR_MIN_FLOOR 1U
#define ELEVATOR_MAX_FLOOR 4U

/* ── FSM state values  (stored in Ipc_ElevatorState.State as uint8) ── */
#define FSM_IDLE         0U
#define FSM_MOVING_UP    1U
#define FSM_MOVING_DOWN  2U
#define FSM_DOORS_OPEN   3U
#define FSM_EMERGENCY    4U
#define FSM_DOORS_CLOSING 5U   // transient state: motor stopped, waiting 1 tick


/* ── Direction values  (stored in Ipc_ElevatorState.Direction as uint8) */
#define DIR_NONE 0U
#define DIR_UP 1U
#define DIR_DOWN 2U

/* ── Live local state — volatile because ISRs write it at any time ── */
extern volatile Ipc_ElevatorState localState;

/* ── Public API ───────────────────────────────────────────────────── */
void Elevator_FSM_Init(uint8 startFloor);
void Elevator_FSM_Tick(void);
void Elevator_FSM_RequestFloor(uint8 floor);
void Elevator_FSM_SetEmergency(void);
Ipc_ElevatorState Elevator_FSM_GetSnapshot(void);

#endif /* ELEVATOR_FSM_H */