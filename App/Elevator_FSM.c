/*
 * Elevator_FSM.c
 *
 * Minimal FSM surface for the current workspace.
 *
 * The full transition logic is owned by the application team, but the
 * system and dispatcher layers need a concrete state object and the public
 * symbols below so the firmware links cleanly and the IPC/dispatcher code
 * can observe a coherent local elevator snapshot.
 */

#include "Elevator_FSM.h"
#include "Lib/Utils.h"

Ipc_ElevatorState localState = {
    .State = FSM_IDLE,
    .Floor = ELEVATOR_MIN_FLOOR,
    .Direction = DIR_NONE,
    .QueueHi = 0U,
    .QueueLo = 0U,
    .Flags = 0U
};

void Elevator_FSM_Init(uint8 startFloor)
{
    if (startFloor < ELEVATOR_MIN_FLOOR) {
        startFloor = ELEVATOR_MIN_FLOOR;
    } else if (startFloor > ELEVATOR_MAX_FLOOR) {
        startFloor = ELEVATOR_MAX_FLOOR;
    }

    localState.State = FSM_IDLE;
    localState.Floor = startFloor;
    localState.Direction = DIR_NONE;
    localState.QueueHi = 0U;
    localState.QueueLo = 0U;
    localState.Flags = 0U;
}

void Elevator_FSM_Tick(void)
{
    /* Placeholder: the concrete motion/state transition logic is not part of
     * the Member 4 timing/telemetry slice. The local snapshot remains valid
     * for dispatcher and telemetry consumers.
     */
}

void Elevator_FSM_RequestFloor(uint8 floor)
{
    if (floor < ELEVATOR_MIN_FLOOR || floor > ELEVATOR_MAX_FLOOR) {
        return;
    }

    if (floor <= 2U) {
        localState.QueueLo |= (uint8)(1U << (floor - 1U));
    } else {
        localState.QueueHi |= (uint8)(1U << (floor - 3U));
    }
}

void Elevator_FSM_SetFault(void)
{
    localState.State = FSM_FAULT;
    localState.Direction = DIR_NONE;
}

Ipc_ElevatorState Elevator_FSM_GetSnapshot(void)
{
    Ipc_ElevatorState copy;

    Enter_Critical();
    copy = localState;
    Exit_Critical();

    return copy;
}