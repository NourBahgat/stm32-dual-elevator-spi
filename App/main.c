/**
 * main.c
 *
 * Entry point for the dual-elevator STM32F401 firmware.
 *
 * Architecture: Event-driven, non-blocking, real-time
 *   - ISRs set flags only (<2 µs)
 *   - Main loop polls flags and dispatches application logic
 *   - No blocking delays, busy-wait loops, or polling in critical paths
 *
 * System Startup Sequence:
 *   1. System_Init()        → Initialize all drivers and ISRs
 *   2. Main event loop      → Poll flags, run application
 *
 * Timing:
 *   SysTick:   1 ms   (system clock)
 *   TIM5:      500 ms (FSM, Dispatcher, Telemetry)
 *   Motor PWM: 10 kHz (hardware-driven, autonomous)
 */

#include "Dispatcher.h"
#include "Elevator_FSM.h"
#include "HallCallButtons.h"
#include "Ipc.h"
#include "Pwm.h"
#include "System.h"
#include "Timer.h"
#include "Telemetry.h"
#include "pwm_scheduler.h"

/* ========================================================================
 * Global Motor Control Instances
 * Instance-based design allows flexible, decoupled control
 * ======================================================================== */

/* Motor A: Master's own elevator */
static PwmContext_t motor_a;

/* Motor B: Slave elevator (state tracking on master side) */
static PwmContext_t motor_b;

/* ========================================================================
 * main
 *
 * Entry point. Initialize system, then run event loop forever.
 * ======================================================================== */
int main(void) {
  /* ---- System Initialization ---------------------------------------- */
  System_Init();

  /* ---- Application Layer Initialization ----------------------------- */

  /* PWM Scheduler contexts */
  PwmScheduler_Init(&motor_a, TIMER2, PWM_CHANNEL_1);
  PwmScheduler_Init(&motor_b, TIMER2, PWM_CHANNEL_2);

  /* Elevator FSM (start at floor 1) */
  Elevator_FSM_Init(1);

  /* IPC (inter-processor communication via SPI) */
  IPC_Init();

  /* Dispatcher (elevator assignment algorithm) */
  Dispatcher_Init();

  /* Hall call buttons (EXTI interrupts) */
  HallCallButtons_Init();

  /* Telemetry system */
  Telemetry_Init();

  /* ---- Main Event Loop ---------------------------------------------- */
  while (1) {
    System_MainLoop_Iteration();

    /* The above handles all periodic tasks via flags.
     * Below is where you might add lower-priority background work
     * (e.g., LED blink, debug output, watchdog, etc.)
     */
  }

  return 0;
}

/**
 * 10 ms hook: PWM ramping is advanced here so the scheduler stays out of
 * interrupt context.
 */
void Application_10ms_Tick(void) {
  PwmScheduler_Tick(&motor_a);
  PwmScheduler_Tick(&motor_b);
}

/**
 * 100 ms hook: comms health check stays in main-loop context.
 */
void Application_100ms_Tick(void) { Dispatcher_CheckCommFault(system_time_ms); }

/**
 * 500 ms hook: run application decisions first, then queue telemetry.
 */
void Application_500ms_Tick(void) {
  static TelemetryFrame_t frame;
  Ipc_ElevatorState local_snapshot = Elevator_FSM_GetSnapshot();
  Ipc_ElevatorState remote_snapshot;
  
  /* Get the remote state from IPC */
  IPC_GetRemoteStateSnapshot(&remote_snapshot);

  uint8 telemetry_flags = local_snapshot.Flags;

  /* Tick the local FSM and the global Dispatcher */
  Elevator_FSM_Tick();
  Dispatcher_Tick();

  /* Update the local motor PWM based on the new FSM state */
  PwmScheduler_UpdateState(&motor_a, (ElevatorState_t)local_snapshot.State);
  
  /* Update the remote motor PWM (tracking) based on the received IPC state */
  PwmScheduler_UpdateState(&motor_b, (ElevatorState_t)remote_snapshot.State);

  if (local_snapshot.Direction == DIR_UP) {
    telemetry_flags |= 0x01U;
  } else if (local_snapshot.Direction == DIR_DOWN) {
    telemetry_flags |= 0x02U;
  }

  if (Ipc_CommFault != 0U) {
    telemetry_flags |= 0x80U;
  }

  /* Build the 6-byte telemetry frame from the live application state. */
  Telemetry_BuildFrame(&frame, local_snapshot.State, local_snapshot.Floor,
                       motor_a.current_duty, telemetry_flags);

  /* Queue the frame; Telemetry_Tick() drains it from the main loop. */
  Telemetry_SendDMA(&frame);
}

/**
 * Background hook: keeps low-priority consumers running without blocking.
 */
void Application_BackgroundTick(void) {
  HallCallButtons_Process();
  Telemetry_Tick();
}
