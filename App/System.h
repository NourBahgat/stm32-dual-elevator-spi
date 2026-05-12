/**
 * System.h
 *
 * System-level scheduler and time-base management.
 *
 * Responsibilities:
 *   - Manage SysTick (1 ms system clock)
 *   - Manage TIM5 (500 ms periodic event)
 *   - Generate event flags for main loop
 *   - No application logic (only flag setting)
 *
 * Architecture: System Layer (between drivers and application)
 */

#ifndef SYSTEM_H
#define SYSTEM_H

#include "Std_Types.h"
#include "Timer.h"

/* ========================================================================
 * Global Time Counter
 * Updated by SysTick ISR every 1 ms
 * ======================================================================== */
extern volatile uint32 system_time_ms;

/* ========================================================================
 * Event Flags
 * Set by ISRs, cleared by application
 * ======================================================================== */
extern volatile uint8 flag_1ms_tick;
extern volatile uint8 flag_10ms_tick;
extern volatile uint8 flag_100ms_tick;
extern volatile uint8 flag_500ms_tick;

/* ========================================================================
 * System_Init
 *
 * Initialize all drivers and interrupt handlers:
 *   1. Rcc (clock tree)
 *   2. Gpio (pin configuration)
 *   3. Nvic (interrupt priorities)
 *   4. Timer (SysTick + TIM5)
 *   5. Usart (UART for telemetry)
 *   6. Pwm (motor control outputs)
 *   7. Enable global interrupts
 *
 * After this, the system is running with all ISRs active.
 * ======================================================================== */
void System_Init(void);

/* ========================================================================
 * Application hooks
 *
 * These are invoked from the main loop only. The System layer stays
 * deterministic by doing nothing more than flag management and callback
 * dispatch.
 * ======================================================================== */
void Application_10ms_Tick(void);
void Application_100ms_Tick(void);
void Application_500ms_Tick(void);
void Application_BackgroundTick(void);

/* ========================================================================
 * System_Tick_500ms
 *
 * Called by main loop when flag_500ms_tick is set.
 * This is where heavy application logic runs:
 *   - Elevator_FSM_Tick()
 *   - Dispatcher_Tick()
 *   - Telemetry_SendDMA()
 *
 * NOT called from ISR — runs in main loop context (safe).
 * ======================================================================== */
void System_Tick_500ms(void);

/**
 * Single iteration of the event loop. Called from while(1) in main().
 */
void System_MainLoop_Iteration(void);

#endif /* SYSTEM_H */
