/*
 * Elevator_FSM.c  —  Member 3: FSM Design & Concurrency Safety
 *
 * RESPONSIBILITIES (Rubric: 20% FSM Design + 20% Race Condition Handling):
 *
 *  1. Finite State Machine with 5 states:
 *       IDLE → MOVING_UP / MOVING_DOWN → DOOR_OPEN → DOOR_CLOSE → IDLE
 *       Any state → EMERGENCY (on emergency-stop button press)
 *
 *  2. EXTI Interrupt handling (all callbacks registered here):
 *       Cabin Buttons  : PB6–PB9  → request a floor
 *       Floor Sensors  : PB10–PB13 → update current floor position
 *       Emergency Stop : PA15     → highest NVIC priority (0)
 *
 *  3. Concurrency Safety:
 *       • volatile   on every variable shared between ISRs and main loop
 *       • Enter_Critical() / Exit_Critical() around every multi-byte
 *         read-modify-write on localState
 *       • Atomic single-byte flag updates inside ISRs (no critical section
 *         needed for single-byte writes on Cortex-M4)
 *
 * HOW IT CONNECTS TO THE REST OF THE PROJECT:
 *   - IPC.c reads localState via Elevator_FSM_GetSnapshot() (atomic copy)
 *   - Dispatcher.c reads the snapshot to run the task allocation algorithm
 *   - main.c calls Elevator_FSM_Tick() periodically from the scheduler
 */

#include "Elevator_FSM.h"
#include "Exti.h"      /* Exti_Init(), Exti_Enable()    */
#include "Gpio.h"      /* Gpio_Init()                   */
#include "Nvic.h"      /* Nvic_SetPriority()            */
#include "Usart.h"     /* Usart1_TransmitString         */
#include "Utils.h"     /* Enter_Critical, Exit_Critical */

/* System millisecond counter from System.c (incremented by SysTick every 1ms) */
extern volatile uint32 system_time_ms;

/* ══════════════════════════════════════════════════════════════════════════
 * SHARED STATE  —  marked volatile because ISRs write Floor, QueueLo,
 *                  QueueHi, State and Direction at any moment.
 * The Dispatcher and IPC layers always obtain a copy via
 * Elevator_FSM_GetSnapshot(), which disables interrupts for the duration
 * of the 6-byte struct copy to prevent torn reads.
 * ══════════════════════════════════════════════════════════════════════════ */
volatile Ipc_ElevatorState localState = {
    .State     = FSM_IDLE,
    .Floor     = ELEVATOR_MIN_FLOOR,
    .Direction = DIR_NONE,
    .QueueHi   = 0U,
    .QueueLo   = 0U,
    .Flags     = 0U
};

/* ══════════════════════════════════════════════════════════════════════════
 * DOOR TIMER
 * Counted in Elevator_FSM_Tick() calls (each tick = 10 ms).
 * Doors stay open for DOOR_OPEN_TICKS ticks = 2 seconds.
 * ══════════════════════════════════════════════════════════════════════════ */
#define DOOR_OPEN_MS  50U
static volatile uint32 door_open_start_ms = 0U;

/* For terminal debugging */
static uint8 last_logged_state = 0xFFU;
static const char* state_names[] = {
    "IDLE", 
    "MOVING_UP", 
    "MOVING_DOWN", 
    "DOORS_OPEN", 
    "EMERGENCY"
};

/* Standard Cortex-M system reset via SCB->AIRCR. Equivalent to NVIC_SystemReset(). */
#define SCB_AIRCR_REG  REG32(0xE000ED0CU)

static void Trigger_SystemReset(void)
{
    SCB_AIRCR_REG = (0x5FAU << 16) | (1U << 2);
    while (1) {
        /* wait for reset */
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * PRIVATE HELPERS — floor request queue
 * ══════════════════════════════════════════════════════════════════════════ */

/* Check whether a floor is pending in the request queue */
static uint8 Queue_Has(uint8 floor)
{
    if (floor <= 2U) {
        return (localState.QueueLo & (uint8)(1U << (floor - 1U))) != 0U;
    }
    return (localState.QueueHi & (uint8)(1U << (floor - 3U))) != 0U;
}

/* Remove a floor from the request queue  */
static void Queue_Clear(uint8 floor)
{
    if (floor <= 2U) {
        localState.QueueLo &= ~(uint8)(1U << (floor - 1U));
    } else {
        localState.QueueHi &= ~(uint8)(1U << (floor - 3U));
    }
}

/* Find the nearest pending request in the CURRENT direction of travel.
 * Returns 0 if the queue is empty or no floor is pending in that direction. */
static uint8 Queue_NextInDirection(void)
{
    if (localState.Direction == DIR_UP) {

        /* Scan floors above current floor */
        for (uint8 f = localState.Floor + 1U;
             f <= ELEVATOR_MAX_FLOOR;
             f++)
        {
            if (Queue_Has(f)) {
                return f;
            }
        }

    } else if (localState.Direction == DIR_DOWN) {

        /* Scan floors below current floor */
        for (sint8 f = (sint8)localState.Floor - 1;
             f >= (sint8)ELEVATOR_MIN_FLOOR;
             f--)
        {
            if (Queue_Has((uint8)f)) {
                return (uint8)f;
            }
        }
    }

    return 0U;
}

/* Find ANY pending request — used when idle to pick a direction */
static uint8 Queue_NextAny(uint8 *out_direction)
{
    /* Prefer upward requests first, then downward */
    for (uint8 f = ELEVATOR_MIN_FLOOR; f <= ELEVATOR_MAX_FLOOR; f++) {
        if (Queue_Has(f) && f > localState.Floor) {
            *out_direction = DIR_UP;
            return f;
        }
    }
    for (sint8 f = ELEVATOR_MAX_FLOOR;
        f >= ELEVATOR_MIN_FLOOR;
        f--)
    {
        if (Queue_Has((uint8)f) && f < localState.Floor) {
            *out_direction = DIR_DOWN;
            return (uint8)f;
        }
    }
    /* Request exactly at the current floor (e.g. door re-open) */
    if (Queue_Has(localState.Floor)) {
        *out_direction = DIR_NONE;
        return localState.Floor;
    }
    return 0U;  /* queue is empty */
}

/* ══════════════════════════════════════════════════════════════════════════
 * ISR CALLBACKS  —  keep these VERY short (<2 µs)
 *
 * Rule: ISRs only SET flags / update single bytes; the FSM Tick() reads
 * them.  Single-byte writes on Cortex-M4 are atomic, so no critical
 * section is needed inside the ISR itself.
 * ══════════════════════════════════════════════════════════════════════════ */

/* ── Cabin floor buttons (PB6–PB9) ─────────────────────────────────────── */
static void ISR_Cabin_F1(void) { Elevator_FSM_RequestFloor(1U); }
static void ISR_Cabin_F2(void) { Elevator_FSM_RequestFloor(2U); }
static void ISR_Cabin_F3(void) { Elevator_FSM_RequestFloor(3U); }
static void ISR_Cabin_F4(void) { Elevator_FSM_RequestFloor(4U); }

/* ── Floor sensors (PB10–PB13): update position when car arrives ────────── */
/* These update a single uint8 field — atomic on Cortex-M4, no lock needed */
static void ISR_Sensor_F1(void) { localState.Floor = 1U; Usart1_TransmitString("[Sensor] Arrived Floor 1\r\n"); }
static void ISR_Sensor_F2(void) { localState.Floor = 2U; Usart1_TransmitString("[Sensor] Arrived Floor 2\r\n"); }
static void ISR_Sensor_F3(void) { localState.Floor = 3U; Usart1_TransmitString("[Sensor] Arrived Floor 3\r\n"); }
static void ISR_Sensor_F4(void) { localState.Floor = 4U; Usart1_TransmitString("[Sensor] Arrived Floor 4\r\n"); }

/* ── Emergency Stop (PA15) — NVIC priority 0 (highest) ─────────────────── */
/*
 * CONCURRENCY NOTE:
 * This ISR writes two bytes (State + Direction) — NOT atomic.
 * We disable interrupts inside the ISR to make this update atomic.
 * This is safe because the emergency stop ISR itself has the highest
 * priority and cannot be preempted by any other ISR in this system.
 */
static void ISR_EmergencyStop(void)
{
    Enter_Critical();               /* disable ALL interrupts               */
    localState.State     = FSM_EMERGENCY;
    localState.Direction = DIR_NONE;
    Exit_Critical();                /* re-enable interrupts                 */
}

/* ── Reset Button (PB11) — software reset after emergency ─────────────── */
static void ISR_ResetButton(void)
{
    if (localState.State == FSM_EMERGENCY) {
        Trigger_SystemReset();
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * PUBLIC API
 * ══════════════════════════════════════════════════════════════════════════ */

/*
 * Elevator_FSM_Init
 * -----------------
 * Call ONCE at startup from main.c before entering the event loop.
 *
 * 1. Initialises the shared state struct (critical section because the
 *    SysTick ISR might already be running at this point).
 * 2. Configures GPIO input pins for buttons and sensors.
 * 3. Registers EXTI callbacks and enables all interrupt lines.
 * 4. Sets the Emergency-Stop interrupt to NVIC priority 0 (highest).
 */
void Elevator_FSM_Init(uint8 startFloor)
{
    /* Clamp start floor to valid range */
    if (startFloor < ELEVATOR_MIN_FLOOR) startFloor = ELEVATOR_MIN_FLOOR;
    if (startFloor > ELEVATOR_MAX_FLOOR) startFloor = ELEVATOR_MAX_FLOOR;

    /* ── Reset shared state atomically ───────────────────────────────── */
    Enter_Critical();
    localState.State     = FSM_IDLE;
    localState.Floor     = startFloor;
    localState.Direction = DIR_NONE;
    localState.QueueHi   = 0U;
    localState.QueueLo   = 0U;
    localState.Flags     = 0U;
    door_open_start_ms   = 0U;
    Exit_Critical();

    /* ── GPIO: Cabin Buttons on PB6–PB9 (input, internal pull-up) ────── */
    Gpio_Init(GPIO_B, 6U,  GPIO_INPUT, GPIO_PULL_UP);
    Gpio_Init(GPIO_B, 7U,  GPIO_INPUT, GPIO_PULL_UP);
    Gpio_Init(GPIO_B, 8U,  GPIO_INPUT, GPIO_PULL_UP);
    Gpio_Init(GPIO_B, 9U,  GPIO_INPUT, GPIO_PULL_UP);

    /* ── GPIO: Floor Sensors (PB10, PB14, PB12, PB13) ─────────────────── */
    Gpio_Init(GPIO_B, 10U, GPIO_INPUT, GPIO_PULL_UP);
    Gpio_Init(GPIO_B, 14U, GPIO_INPUT, GPIO_PULL_UP); /* Moved from PB11 */
    Gpio_Init(GPIO_B, 12U, GPIO_INPUT, GPIO_PULL_UP);
    Gpio_Init(GPIO_B, 13U, GPIO_INPUT, GPIO_PULL_UP);

    /* ── GPIO: Reset Button on PB11 (input, internal pull-up) ─────────── */
    Gpio_Init(GPIO_B, 11U, GPIO_INPUT, GPIO_PULL_UP);

    /* ── GPIO: Emergency Stop on PA15 (input, internal pull-up) ──────── */
    Gpio_Init(GPIO_A, 15U, GPIO_INPUT, GPIO_PULL_UP);

    /* ── EXTI: Cabin Buttons — falling edge (button press = low) ──────── */
    Exti_Init(EXTI_LINE_6,  EXTI_PORT_B, EXTI_EDGE_FALLING, ISR_Cabin_F1);
    Exti_Init(EXTI_LINE_7,  EXTI_PORT_B, EXTI_EDGE_FALLING, ISR_Cabin_F2);
    Exti_Init(EXTI_LINE_8,  EXTI_PORT_B, EXTI_EDGE_FALLING, ISR_Cabin_F3);
    Exti_Init(EXTI_LINE_9,  EXTI_PORT_B, EXTI_EDGE_FALLING, ISR_Cabin_F4);

    /* ── EXTI: Floor Sensors — falling edge (sensor triggers = low) ───── */
    Exti_Init(EXTI_LINE_10, EXTI_PORT_B, EXTI_EDGE_FALLING, ISR_Sensor_F1);
    Exti_Init(EXTI_LINE_14, EXTI_PORT_B, EXTI_EDGE_FALLING, ISR_Sensor_F2); /* PB14 */
    Exti_Init(EXTI_LINE_12, EXTI_PORT_B, EXTI_EDGE_FALLING, ISR_Sensor_F3);
    Exti_Init(EXTI_LINE_13, EXTI_PORT_B, EXTI_EDGE_FALLING, ISR_Sensor_F4);

    /* ── EXTI: Reset Button — falling edge ─────────────────────────────── */
    Exti_Init(EXTI_LINE_11, EXTI_PORT_B, EXTI_EDGE_FALLING, ISR_ResetButton);

    /* ── EXTI: Emergency Stop — falling edge ───────────────────────────── */
    Exti_Init(EXTI_LINE_15, EXTI_PORT_A, EXTI_EDGE_FALLING, ISR_EmergencyStop);

    /* ── Enable EXTI lines in NVIC ────────────────────────────────────── */
    Exti_Enable(EXTI_LINE_6);   /* Cabin F1 */
    Exti_Enable(EXTI_LINE_7);   /* Cabin F2 */
    Exti_Enable(EXTI_LINE_8);   /* Cabin F3 */
    Exti_Enable(EXTI_LINE_9);   /* Cabin F4 */
    Exti_Enable(EXTI_LINE_10);  /* Sensor F1 */
    Exti_Enable(EXTI_LINE_14);  /* Sensor F2 (PB14) */
    Exti_Enable(EXTI_LINE_12);  /* Sensor F3 */
    Exti_Enable(EXTI_LINE_13);  /* Sensor F4 */
    Exti_Enable(EXTI_LINE_11);  /* Reset Button */
    Exti_Enable(EXTI_LINE_15);  /* Emergency Stop */

    /*
     * ── NVIC Priority: Emergency Stop = 0 (highest possible) ────────────
     *
     * STM32F401 IRQ number for EXTI15_10 is 40.
     * Priority 0 means it preempts ALL other ISRs including SysTick (15)
     * and TIM5 (50). The emergency stop MUST be serviced instantly.
     */
    Nvic_SetPriority(40U, 0U);  /* EXTI15_10_IRQn — Emergency Stop */
}

/*
 * Elevator_FSM_Tick
 * -----------------
 * Called every 10 ms from Application_0ms_Tick() in main.c
 * Advances the FSM one step.
 *
 * STATE MACHINE DIAGRAM:
 *
 *   ┌─────────────────────────────────────────────────────────┐
 *   │  Any state ──[Emergency button]──► EMERGENCY (FSM_FAULT)│
 *   └─────────────────────────────────────────────────────────┘
 *
 *   IDLE ──[request exists]──► MOVING_UP or MOVING_DOWN
 *     │                              │
 *     │   [request at current floor] │ [floor sensor fires → Floor updated]
 *     └──────────────────────────────┘
 *                                    │ [request at new Floor]
 *                                    ▼
 *                              DOOR_OPEN ──[2 s timer]──► DOOR_CLOSE
 *                                                               │
 *                                                               ▼
 *                                                             IDLE
 */
void Elevator_FSM_Tick(void)
{
    /* Log state transitions for terminal debugging */
    if (localState.State != last_logged_state) {
        Usart1_TransmitString("[FSM] State: ");
        Usart1_TransmitString(state_names[localState.State]);
        Usart1_TransmitString("\r\n");
        last_logged_state = localState.State;
    }

    /* EMERGENCY is a terminal state — only a system reset clears it */
    if (localState.State == FSM_EMERGENCY) {
        return;
    }

    switch (localState.State) {

    /* ── IDLE: look for a pending request ──────────────────────────── */
    case FSM_IDLE:
    {
        uint8 dir = DIR_NONE;
        uint8 target = Queue_NextAny(&dir);

        if (target == 0U) {
            /* Queue is empty — stay idle, nothing to do */
            break;
        }

        if (target == localState.Floor) {
            /* Request is AT the current floor: open doors immediately */
            Enter_Critical();
            Queue_Clear(localState.Floor);
            localState.State     = FSM_DOORS_OPEN;
            localState.Direction = DIR_NONE;
            door_open_start_ms   = system_time_ms;   /* start 2-second timer */
            Exit_Critical();
        } else {
            /* Request is on another floor: start moving */
            Enter_Critical();
            localState.State     = (dir == DIR_UP) ? FSM_MOVING_UP
                                                    : FSM_MOVING_DOWN;
            localState.Direction = dir;
            Exit_Critical();
        }
        break;
    }

    /* ── MOVING_UP / MOVING_DOWN: check if we reached a requested floor */
    case FSM_MOVING_UP:
    case FSM_MOVING_DOWN:
    {
        if (Queue_Has(localState.Floor)) {
            Enter_Critical();
            Queue_Clear(localState.Floor);
            localState.State     = FSM_DOORS_OPEN;
            localState.Direction = DIR_NONE;
            door_open_start_ms   = system_time_ms;   /* start 2-second timer */
            Exit_Critical();
            break;
        }

        /* Check if any request remains in the current travel direction */
        uint8 next = Queue_NextInDirection();
        if (next == 0U) {
            uint8 dir = DIR_NONE;
            uint8 anyTarget = Queue_NextAny(&dir);
            Enter_Critical();
            if (anyTarget == 0U) {
                localState.State     = FSM_IDLE;
                localState.Direction = DIR_NONE;
            } else {
                localState.State     = (dir == DIR_UP) ? FSM_MOVING_UP
                                                        : FSM_MOVING_DOWN;
                localState.Direction = dir;
            }
            Exit_Critical();
        }
        break;
    }

    /* ── DOORS_OPEN: hold for 2 seconds then transition to IDLE ─────── */
    case FSM_DOORS_OPEN:
        /* Use absolute timestamp — immune to tick-rate variation in Proteus */
        if ((system_time_ms - door_open_start_ms) >= DOOR_OPEN_MS) {
            Usart1_TransmitString("[FSM] Doors closed -> IDLE\r\n");
            Enter_Critical();
            localState.State     = FSM_IDLE;
            localState.Direction = DIR_NONE;
            door_open_start_ms   = 0U;
            Exit_Critical();
        }
        break;

    default:
        Enter_Critical();
        localState.State = FSM_IDLE;
        Exit_Critical();
        break;
    }
}

/*
 * Elevator_FSM_RequestFloor
 * -------------------------
 * Called from cabin-button ISRs OR from the Dispatcher when the Slave
 * receives a dispatch command via IPC.
 *
 * RACE CONDITION PROTECTION:
 * QueueLo and QueueHi are shared between this function (called from an
 * ISR) and Elevator_FSM_Tick() (called from the main loop).
 * A read-modify-write on a byte (|=) is NOT atomic — it compiles to
 * three instructions (load, OR, store). Enter_Critical() prevents the
 * main loop from reading a half-updated byte.
 */
void Elevator_FSM_RequestFloor(uint8 floor)
{
    if (floor < ELEVATOR_MIN_FLOOR || floor > ELEVATOR_MAX_FLOOR) {
        return;  /* Ignore out-of-range requests */
    }

    Enter_Critical();                       /* LOCK  */
    if (floor <= 2U) {
        localState.QueueLo |= (uint8)(1U << (floor - 1U));
    } else {
        localState.QueueHi |= (uint8)(1U << (floor - 3U));
    }
    Exit_Critical();                        /* UNLOCK */
}

/**
 * Elevator_FSM_SetEmergency
 * ---------------------
 * Called by IPC layer when a communication timeout is detected, or
 * directly by the emergency ISR (via the ISR_EmergencyStop callback).
 */
void Elevator_FSM_SetEmergency(void)
{
    Enter_Critical();
    localState.State     = FSM_EMERGENCY;
    localState.Direction = DIR_NONE;
    Exit_Critical();
}

/*
 * Elevator_FSM_GetSnapshot
 * ------------------------
 * Returns an ATOMIC copy of the full 6-byte localState struct.
 *
 * WHY CRITICAL SECTION HERE:
 * The struct is 6 bytes. Cortex-M4 cannot copy 6 bytes in a single
 * instruction. Without Enter_Critical(), an ISR could update localState
 * after we've copied the first 2 bytes but before we've copied the last
 * 4, giving the caller a "torn" / inconsistent snapshot.
 *
 * The IPC layer calls this every 50 ms to pack a SPI frame; the
 * Dispatcher calls it on every hallway button press.
 */
Ipc_ElevatorState Elevator_FSM_GetSnapshot(void)
{
    Ipc_ElevatorState copy;

    Enter_Critical();                       /* LOCK  */
    copy = *(Ipc_ElevatorState *)&localState;
    Exit_Critical();                        /* UNLOCK */

    return copy;
}