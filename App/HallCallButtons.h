#ifndef HALL_CALL_BUTTONS_H
#define HALL_CALL_BUTTONS_H

#include "Std_Types.h"

/*
 * Hardware mapping (6 buttons, active-LOW on falling edge with pull-up):
 *
 *   Floor 1 UP   → PC0  (EXTI_LINE_0)
 *   Floor 2 UP   → PC1  (EXTI_LINE_1)
 *   Floor 2 DOWN → PC2  (EXTI_LINE_2)
 *   Floor 3 UP   → PC3  (EXTI_LINE_3)
 *   Floor 3 DOWN → PC4  (EXTI_LINE_4)
 *   Floor 4 DOWN → PC5  (EXTI_LINE_5)
 *
 * NOTE: PA0/PA1 are reserved for TIM2 CH1/CH2 PWM (motor LEDs).
 *
 * Each ISR writes one HallCall entry into a volatile ring buffer.
 * The main loop drains the buffer and calls Dispatcher_AssignElevator().
 * No FSM functions are called from within any ISR.
 */

/* ── Pending-call ring buffer ─────────────────────────────────────── */
#define HALL_CALL_BUF_SIZE   8U   /* power-of-2 keeps wrap-around cheap */

typedef struct {
    uint8 Floor;      /* 1-4 */
    uint8 Direction;  /* DIR_UP / DIR_DOWN  (from Elevator_FSM.h) */
} HallCall;

typedef struct {
    volatile HallCall  Calls[HALL_CALL_BUF_SIZE];
    volatile uint8     Head;   /* write index (ISR advances this) */
    volatile uint8     Tail;   /* read  index (main loop advances) */
} HallCallRingBuf;

/* Shared between ISR and main loop — defined in HallCallButtons.c */
extern HallCallRingBuf HallCallBuf;

/* ── Public API ───────────────────────────────────────────────────── */

/**
 * @brief  Configure all 6 hallway GPIO pins as inputs and register
 *         their EXTI callbacks.  Call once at startup before enabling
 *         global interrupts.
 */
void HallCallButtons_Init(void);

/**
 * @brief  Drain the pending-call ring buffer and dispatch each call.
 *
 *  Must be called from the main loop (not from an ISR).
 *  Calls Dispatcher_AssignElevator() for every pending entry.
 */
void HallCallButtons_Process(void);

#endif /* HALL_CALL_BUTTONS_H */