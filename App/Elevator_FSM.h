#ifndef ELEVATOR_FSM_H
#define ELEVATOR_FSM_H

#include "Std_Types.h"
#include "Ipc.h"

/* ── Floor range ──────────────────────────────────────────────────── */
#define ELEVATOR_MIN_FLOOR  1U
#define ELEVATOR_MAX_FLOOR  4U

/* ── FSM state values  (stored in Ipc_ElevatorState.State as uint8) ── */
#define FSM_IDLE         0U
#define FSM_MOVING_UP    1U
#define FSM_MOVING_DOWN  2U
#define FSM_DOOR_OPEN    3U
#define FSM_DOOR_CLOSE   4U
#define FSM_FAULT        5U

/* ── Direction values  (stored in Ipc_ElevatorState.Direction as uint8) */
#define DIR_NONE  0U
#define DIR_UP    1U
#define DIR_DOWN  2U

/* ── Live local state — defined in Elevator_FSM.c, read by IPC & Dispatcher */
extern Ipc_ElevatorState localState;

/* ── Public API ───────────────────────────────────────────────────── */
void Elevator_FSM_Init(uint8 startFloor);
void Elevator_FSM_Tick(void);
void Elevator_FSM_RequestFloor(uint8 floor);
void Elevator_FSM_SetFault(void);

#endif /* ELEVATOR_FSM_H */