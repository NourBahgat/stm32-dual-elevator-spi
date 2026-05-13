/*
 * Dispatcher.c
 *
 * Master-MCU task-allocation algorithm for the dual-elevator system.
 *
 *  Elevator A = this MCU's own elevator   → localState        (Elevator_FSM.c)
 *  Elevator B = remote slave elevator     → IPC_GetRemoteStateSnapshot()
 * (IPC.c)
 */

#include "Dispatcher.h"
#include "Usart.h"
/* IPC_GetRemoteStateSnapshot() is declared in Ipc.h (included via Dispatcher.h)
 */

/* ── Deferred-call store (single pending call) ────────────────────── */
#define NO_DEFERRED_CALL 0U

static uint8 deferred_floor = NO_DEFERRED_CALL;
static uint8 deferred_direction = DIR_UP;

/* ── Private helpers ──────────────────────────────────────────────── */

/** Unsigned distance between two floor numbers. */
static uint8 floor_dist(uint8 a, uint8 b) {
  return (a >= b) ? (a - b) : (b - a);
}

/**
 * @brief  Returns 1 if elevator ev is moving toward call_floor in
 * call_direction.
 *
 *  Uses only the existing Floor and Direction fields — no TargetFloor needed.
 *  A elevator moving UP qualifies when the call floor is >= its current floor.
 *  A elevator moving DOWN qualifies when the call floor is <= its current
 * floor.
 */
static uint8 moving_toward(const Ipc_ElevatorState *ev, uint8 call_floor,
                           uint8 call_direction) {
  if (ev->State == FSM_MOVING_UP && call_direction == DIR_UP &&
      call_floor >= ev->Floor) {
    return 1U;
  }
  if (ev->State == FSM_MOVING_DOWN && call_direction == DIR_DOWN &&
      call_floor <= ev->Floor) {
    return 1U;
  }
  return 0U;
}

/** Returns 1 if this elevator should be considered available (not faulted). */
static uint8 elevator_available(const Ipc_ElevatorState *ev) {
  return (ev != 0U) && (ev->State != FSM_EMERGENCY);
}

/* ── Core algorithm ───────────────────────────────────────────────── */

Dispatch_Result Dispatcher_AssignElevator(uint8 call_floor,
                                          uint8 call_direction) {
  /* --- Obtain pointers to both elevators' states ---------------------- */
  const Ipc_ElevatorState *A = (const Ipc_ElevatorState *)&localState;

  /* Take an atomic snapshot of B's state to prevent ISR race conditions */
  Ipc_ElevatorState b_snapshot;
  IPC_GetRemoteStateSnapshot(&b_snapshot);
  const Ipc_ElevatorState *B = &b_snapshot;

  uint8 a_ok = elevator_available(A);
  /* Comm fault: exclude Slave B so Master A handles all calls alone. */
  uint8 b_ok = elevator_available(B) && (Ipc_CommFault == 0U);

  /* Both elevators faulted / unavailable — defer the call */
  if (!a_ok && !b_ok) {
    deferred_floor     = call_floor;
    deferred_direction = call_direction;
    return DISPATCH_NONE;
  }

  /* ── P1: Immediate — elevator already at call floor AND IDLE ──────────
   *        Best case — zero travel required.                         */
  if (a_ok && A->State == FSM_IDLE && A->Floor == call_floor)
    return DISPATCH_A;
  if (b_ok && B->State == FSM_IDLE && B->Floor == call_floor)
    return DISPATCH_B;

  /* ── P2: Perfect Match — moving toward the floor in the same direction
   *        Elevator has not yet passed the call floor.
   *        If both qualify, pick the one closer to the call floor.   */
  {
    uint8 a_toward = a_ok && moving_toward(A, call_floor, call_direction);
    uint8 b_toward = b_ok && moving_toward(B, call_floor, call_direction);

    if (a_toward && b_toward) {
      return (floor_dist(A->Floor, call_floor) <=
              floor_dist(B->Floor, call_floor))
                 ? DISPATCH_A
                 : DISPATCH_B;
    }
    if (a_toward) return DISPATCH_A;
    if (b_toward) return DISPATCH_B;
  }

  /* ── P3: Passed Match — same direction but car has already passed ─────
   *        Lower priority than Perfect Match; fires only when no P2
   *        candidate exists.  Pick the geometrically closer one.    */
  {
    uint8 a_passed =
        a_ok &&
        ((A->State == FSM_MOVING_UP   && call_direction == DIR_UP   &&
          call_floor < A->Floor) ||
         (A->State == FSM_MOVING_DOWN && call_direction == DIR_DOWN &&
          call_floor > A->Floor));
    uint8 b_passed =
        b_ok &&
        ((B->State == FSM_MOVING_UP   && call_direction == DIR_UP   &&
          call_floor < B->Floor) ||
         (B->State == FSM_MOVING_DOWN && call_direction == DIR_DOWN &&
          call_floor > B->Floor));

    if (a_passed && b_passed) {
      return (floor_dist(A->Floor, call_floor) <=
              floor_dist(B->Floor, call_floor))
                 ? DISPATCH_A
                 : DISPATCH_B;
    }
    if (a_passed) return DISPATCH_A;
    if (b_passed) return DISPATCH_B;
  }

  /* ── P4: Nearest IDLE elevator ────────────────────────────────────────
   *        No elevator is en-route; send whichever idle one is closest. */
  {
    uint8 a_idle = a_ok && (A->State == FSM_IDLE);
    uint8 b_idle = b_ok && (B->State == FSM_IDLE);

    if (a_idle && b_idle) {
      return (floor_dist(A->Floor, call_floor) <=
              floor_dist(B->Floor, call_floor))
                 ? DISPATCH_A
                 : DISPATCH_B;
    }
    if (a_idle) return DISPATCH_A;
    if (b_idle) return DISPATCH_B;
  }

  /* ── P5: Opposite Direction — defer rather than assign against travel ──
   *        If every available elevator is moving AWAY from the call
   *        direction, defer and let Dispatcher_Tick() retry once one
   *        finishes and goes IDLE.                                   */
  {
    uint8 a_opposite =
        a_ok &&
        ((A->State == FSM_MOVING_UP   && call_direction == DIR_DOWN) ||
         (A->State == FSM_MOVING_DOWN && call_direction == DIR_UP));
    uint8 b_opposite =
        b_ok &&
        ((B->State == FSM_MOVING_UP   && call_direction == DIR_DOWN) ||
         (B->State == FSM_MOVING_DOWN && call_direction == DIR_UP));

    /* All available elevators are going the wrong way — defer.
     * Logic: defer if (A unavailable OR A opposite) AND (B unavailable OR B opposite).
     * This defers only when all AVAILABLE elevators are moving opposite direction. */
    if ((!a_ok || a_opposite) && (!b_ok || b_opposite)) {
      deferred_floor     = call_floor;
      deferred_direction = call_direction;
      return DISPATCH_NONE;
    }
    /* At least one is NOT going the wrong way — prefer it */
    if (b_opposite && !a_opposite && a_ok) return DISPATCH_A;
    if (a_opposite && !b_opposite && b_ok) return DISPATCH_B;
  }

  /* ── P6: Absolute fallback ─────────────────────────────────────────────
   *        Remaining cases: DOORS_OPEN on both, or mixed states not
   *        covered above.  Assign closest available elevator.        */
  if (!a_ok) return DISPATCH_B;
  if (!b_ok) return DISPATCH_A;

  return (floor_dist(A->Floor, call_floor) <= floor_dist(B->Floor, call_floor))
             ? DISPATCH_A
             : DISPATCH_B;
}

/* ── Module init ──────────────────────────────────────────────────── */

void Dispatcher_Init(void) {
  deferred_floor = NO_DEFERRED_CALL;
  deferred_direction = DIR_UP;
}

/* ── 50 ms tick: retry any deferred call ─────────────────────────── */

void Dispatcher_Tick(void) {
  if (deferred_floor == NO_DEFERRED_CALL)
    return;

  Dispatch_Result r =
      Dispatcher_AssignElevator(deferred_floor, deferred_direction);
  if (r != DISPATCH_NONE) {
      /* Winner found — request the floor on the chosen elevator */
      if (r == DISPATCH_A) {
        Usart1_TransmitString("[Dispatcher] Assigning call to LOCAL (A)\r\n");
        Elevator_FSM_RequestFloor(deferred_floor);
      } else if (r == DISPATCH_B) {
        Usart1_TransmitString("[Dispatcher] Assigning call to SLAVE (B) via SPI\r\n");
        /* Relay to Slave via IPC command channel (Flags byte). */
        ((Ipc_ElevatorState *)&localState)->Flags =
            (uint8)(IPC_FLAG_DISPATCH_REQ |
                    IPC_FLAGS_ENCODE_FLOOR(deferred_floor));
      }
    deferred_floor = NO_DEFERRED_CALL;
  }
}

/*
 * Dispatcher_CheckCommFault — REMOVED.
 *
 * Comm-fault detection and auto-recovery are handled entirely inside
 * IPC.c by the IPC_MasterTick / IPC_OnFrameReceived pair:
 *
 *   IPC_MasterTick()       increments ipc_TimeoutCounter every 50 ms and
 *                          sets Ipc_CommFault = 1 after 5 missed frames.
 *   IPC_OnFrameReceived()  resets ipc_TimeoutCounter = 0 and
 *                          Ipc_CommFault = 0 on every valid SPI frame.
 *
 * A duplicate mechanism here used last_spi_rx_timestamp (which was never
 * written) and could race with and incorrectly clear a genuine fault flag
 * set by IPC_MasterTick.  It has been removed to avoid that race.
 *
 * Callers (Application_100ms_Tick in main.c) have been updated accordingly.
 */