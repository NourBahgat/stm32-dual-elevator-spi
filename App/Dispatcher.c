/*
 * Dispatcher.c
 *
 * Master-MCU task-allocation algorithm for the dual-elevator system.
 *
 *  Elevator A = this MCU's own elevator   → localState        (Elevator_FSM.c)
 *  Elevator B = remote slave elevator     → IPC_GetRemoteStateSnapshot() (IPC.c)
 */

#include "Dispatcher.h"
/* IPC_GetRemoteStateSnapshot() is declared in Ipc.h (included via Dispatcher.h) */

/* ── Deferred-call store (single pending call) ────────────────────── */
#define NO_DEFERRED_CALL  0U

static uint8 deferred_floor     = NO_DEFERRED_CALL;
static uint8 deferred_direction = DIR_UP;

/* ── Private helpers ──────────────────────────────────────────────── */

/** Unsigned distance between two floor numbers. */
static uint8 floor_dist(uint8 a, uint8 b)
{
    return (a >= b) ? (a - b) : (b - a);
}

/**
 * @brief  Returns 1 if elevator ev is moving toward call_floor in call_direction.
 *
 *  Uses only the existing Floor and Direction fields — no TargetFloor needed.
 *  A elevator moving UP qualifies when the call floor is >= its current floor.
 *  A elevator moving DOWN qualifies when the call floor is <= its current floor.
 */
static uint8 moving_toward(const Ipc_ElevatorState *ev,
                            uint8 call_floor, uint8 call_direction)
{
    if (ev->State == FSM_MOVING_UP   &&
        call_direction == DIR_UP     &&
        call_floor >= ev->Floor)
    {
        return 1U;
    }
    if (ev->State == FSM_MOVING_DOWN &&
        call_direction == DIR_DOWN   &&
        call_floor <= ev->Floor)
    {
        return 1U;
    }
    return 0U;
}

/** Returns 1 if this elevator should be considered available (not faulted). */
static uint8 elevator_available(const Ipc_ElevatorState *ev)
{
    return (ev != 0U) && (ev->State != FSM_FAULT);
}

/* ── Core algorithm ───────────────────────────────────────────────── */

Dispatch_Result Dispatcher_AssignElevator(uint8 call_floor, uint8 call_direction)
{
    /* --- Obtain pointers to both elevators' states ---------------------- */
    const Ipc_ElevatorState *A = &localState;

    /* Take an atomic snapshot of B's state to prevent ISR race conditions */
    Ipc_ElevatorState b_snapshot;
    IPC_GetRemoteStateSnapshot(&b_snapshot);
    const Ipc_ElevatorState *B = &b_snapshot;

    uint8 a_ok = elevator_available(A);
    /* 3) Master handles calls itself using A if comms are down */
    uint8 b_ok = elevator_available(B) && (Ipc_CommFault == 0U);

    /* Both elevators faulted / unavailable — defer the call */
    if (!a_ok && !b_ok) {
        deferred_floor     = call_floor;
        deferred_direction = call_direction;
        return DISPATCH_NONE;
    }

    /* ── P1: elevator already at call floor AND IDLE ─────────────────────
     *        Best case — zero travel required.                        */
    if (a_ok && A->State == FSM_IDLE && A->Floor == call_floor) return DISPATCH_A;
    if (b_ok && B->State == FSM_IDLE && B->Floor == call_floor) return DISPATCH_B;

    /* ── P2: elevator moving toward call floor in the same direction ──────
     *        Uses Floor + Direction from Ipc_ElevatorState only.
     *        If both qualify, pick the one closer to the call floor.  */
    {
        uint8 a_toward = a_ok && moving_toward(A, call_floor, call_direction);
        uint8 b_toward = b_ok && moving_toward(B, call_floor, call_direction);

        if (a_toward && b_toward) {
            return (floor_dist(A->Floor, call_floor) <=
                    floor_dist(B->Floor, call_floor))
                   ? DISPATCH_A : DISPATCH_B;
        }
        if (a_toward) return DISPATCH_A;
        if (b_toward) return DISPATCH_B;
    }

    /* ── P3: Nearest IDLE elevator ────────────────────────────────────────
     *        No elevator is en-route; send whichever idle elevator is closest.  */
    {
        uint8 a_idle = a_ok && (A->State == FSM_IDLE);
        uint8 b_idle = b_ok && (B->State == FSM_IDLE);

        if (a_idle && b_idle) {
            return (floor_dist(A->Floor, call_floor) <=
                    floor_dist(B->Floor, call_floor))
                   ? DISPATCH_A : DISPATCH_B;
        }
        if (a_idle) return DISPATCH_A;
        if (b_idle) return DISPATCH_B;
    }

    /* ── P4: Fallback — both elevators going in the opposite direction ────
     *        Assign to geometrically closer elevator; it will queue the
     *        request and service it after reversing.
     *        No round-robin counter — decision is purely state-based.  */
    if (!a_ok) return DISPATCH_B;
    if (!b_ok) return DISPATCH_A;

    return (floor_dist(A->Floor, call_floor) <=
            floor_dist(B->Floor, call_floor))
           ? DISPATCH_A : DISPATCH_B;
}

/* ── Module init ──────────────────────────────────────────────────── */

void Dispatcher_Init(void)
{
    deferred_floor     = NO_DEFERRED_CALL;
    deferred_direction = DIR_UP;
}

/* ── 50 ms tick: retry any deferred call ─────────────────────────── */

void Dispatcher_Tick(void)
{
    if (deferred_floor == NO_DEFERRED_CALL) return;

    Dispatch_Result r = Dispatcher_AssignElevator(deferred_floor,
                                                  deferred_direction);
    if (r != DISPATCH_NONE) {
        /* Winner found — request the floor on the chosen elevator */
        if (r == DISPATCH_A) {
            Elevator_FSM_RequestFloor(deferred_floor);
        } else if (r == DISPATCH_B) {
            /* Relay to Slave via IPC command channel (Flags byte) */
            localState.Flags = (uint8)(IPC_FLAG_DISPATCH_REQ |
                                       IPC_FLAGS_ENCODE_FLOOR(deferred_floor));
        }
        deferred_floor = NO_DEFERRED_CALL;
    }
}

void Dispatcher_CheckCommFault(uint32 current_time_ms)
{
    /* 1) Detect timeout if no valid packet received within 150ms */
    if ((current_time_ms - last_spi_rx_timestamp) > 150U) {
        /* 2) Set a comm_fault flag */
        Ipc_CommFault = 1U;
        
        /* 3) Switch dispatch mode: This is achieved via the condition
         *    b_ok = elevator_available(B) && (Ipc_CommFault == 0U)
         *    inside Dispatcher_AssignElevator(). By forcing B to be
         *    unavailable, the Master handles all calls natively on A. */
    } else {
        /* 4) Recover automatically when valid packets resume */
        if (Ipc_CommFault == 1U) {
            Ipc_CommFault = 0U;
            /* TODO: FSM status check needed here */
        }
    }
}