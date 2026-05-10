/**
 * System.c
 *
 * System-level initialization and ISR handlers.
 *
 * This module bridges the Driver Layer and Application Layer:
 *   - ISRs set flags (1-2 µs execution time)
 *   - Main loop polls flags and executes application logic
 *   - No application code in ISRs
 *
 * ISR Latency Budget (CRITICAL):
 *   SysTick:   < 2 µs  (only increment counter)
 *   TIM5:      < 2 µs  (only set flag)
 *   DMA:       < 2 µs  (only set flag)
 *   EXTI:      < 2 µs  (only queue event)
 */

#include "System.h"
#include "Rcc.h"
#include "Gpio.h"
#include "Nvic.h"
#include "Timer.h"
#include "Pwm.h"
#include "Usart.h"
#include "Exti.h"
#include "Bit_Operations.h"

/* ========================================================================
 * Global Time Counter (SysTick ISR increments this)
 * ======================================================================== */
volatile uint32 system_time_ms = 0;

/* ========================================================================
 * Event Flags (ISRs set these, application clears)
 * ======================================================================== */
volatile uint8 flag_1ms_tick = 0;
volatile uint8 flag_10ms_tick = 0;
volatile uint8 flag_100ms_tick = 0;
volatile uint8 flag_500ms_tick = 0;

/* ========================================================================
 * Scheduler State (main loop tracking)
 * Used to generate 10ms and 100ms ticks from 1ms SysTick
 * ======================================================================== */
static uint32 last_10ms_tick = 0;
static uint32 last_100ms_tick = 0;

/* ========================================================================
 * SysTick_Handler
 *
 * Called every 1 ms by the hardware timer.
 * CRITICAL: Keep this under 2 µs execution time.
 *
 * Responsibilities:
 *   1. Increment system_time_ms
 *   2. Return immediately (let main loop do the work)
 *
 * Do NOT put prescaler logic or state variables here.
 * ======================================================================== */
void SysTick_Handler(void)
{
    system_time_ms++;
    flag_1ms_tick = 1;
}

/* ========================================================================
 * TIM5_IRQHandler
 *
 * Called every 500 ms by TIM5 (configured with ARPE auto-reload).
 * CRITICAL: Keep this under 2 µs execution time.
 *
 * TIM5 is configured as:
 *   PSC = 15,999 (divide 16 MHz by 16,000 → 1 kHz)
 *   ARR = 499    (count 0-499 → overflow every 500 ticks = 500 ms)
 *   ARPE enabled (auto-reload preload)
 *
 * With ARPE, TIM5 automatically reloads ARR at each overflow and
 * continues firing every 500 ms without manual re-enable.
 * ======================================================================== */
void TIM5_IRQHandler(void)
{
    /* Check update interrupt flag (UIF) */
    if (READ_BIT(TIM5->SR, SR_UIF)) {
        TIM5->SR = 0;              /* Clear UIF */
        flag_500ms_tick = 1;       /* Signal application */
        /* NO interrupt disable — TIM5 runs continuously via ARPE */
    }
}

/* ========================================================================
 * System_Scheduler_Task
 *
 * Runs from main loop to generate 10ms and 100ms ticks.
 * This is NOT in an ISR — it's safe to have more complex logic here.
 *
 * Uses simple polling of system_time_ms with saved checkpoints.
 * More deterministic than prescalers in ISR.
 * ======================================================================== */
static void System_Scheduler_Task(void)
{
    /* 10ms tick */
    if (system_time_ms - last_10ms_tick >= 10) {
        last_10ms_tick = system_time_ms;
        flag_10ms_tick = 1;
    }

    /* 100ms tick */
    if (system_time_ms - last_100ms_tick >= 100) {
        last_100ms_tick = system_time_ms;
        flag_100ms_tick = 1;
    }
}

/* ========================================================================
 * System_Tick_500ms
 *
 * Called by the main loop when flag_500ms_tick is set.
 * This is the application-layer 500 ms scheduling point.
 * ======================================================================== */
void System_Tick_500ms(void)
{
    Application_500ms_Tick();
}

/* ========================================================================
 * System_Init
 *
 * One-time initialization of all subsystems.
 * Called from main() before entering the event loop.
 *
 * Order is critical:
 *   1. Clock tree (enables peripherals)
 *   2. GPIO (configure pins)
 *   3. Interrupt priorities (NVIC)
 *   4. Drivers (Timer, UART, PWM, EXTI)
 *   5. ISR enable
 *   6. Start timers
 * ======================================================================== */
void System_Init(void)
{
    /* ---- 1. Clock Tree ------------------------------------------------ */
    Rcc_Init();  /* APB1, APB2, HSI configuration */

    /* ---- 2. GPIO Configuration ---------------------------------------- */
    Gpio_Init();  /* All pins: buttons, LEDs, motor outputs, UART, SPI, etc. */

    /* ---- 3. NVIC Priority Configuration ------------------------------- */
    Nvic_Init();  /* Set interrupt priorities (SysTick > TIM5 > EXTI) */

    /* ---- 4. Driver Initialization ------------------------------------- */

    /* UART for telemetry */
    Usart1_Init();

    /* Timer: TIM5 for 500ms periodic event */
    Timer_Init(TIMER5, 15999, 499);  /* 500 ms period */

    /* PWM: TIM2 for motor control (10 kHz)
     * 16 MHz / (15 + 1) = 1 MHz timer clock
     * 1 MHz / (99 + 1) = 10 kHz PWM period
     * Duty mapping remains fixed-point: CCR = (DutyPercent * ARR) / 100
     */
    Pwm_Init(TIMER2, PWM_CHANNEL_1, 15, 99);   /* CH1 = Motor A */
    Pwm_Init(TIMER2, PWM_CHANNEL_2, 15, 99);   /* CH2 = Motor B */
    Pwm_Start(TIMER2, PWM_CHANNEL_1);
    Pwm_Start(TIMER2, PWM_CHANNEL_2);

    /* External interrupts (hall call buttons) */
    /* Exti_Init() — if implemented in your Exti driver */

    /* ---- 5. Enable ISRs in NVIC -------------------------------------- */
    Nvic_EnableIrq(TIM5_IRQn);
    /* USART1_IRQn already enabled in Usart1_Init() */
    /* EXTI ISRs enabled in Exti_Init() or per-pin */

    /* ---- 6. Start Periodic Timers ------------------------------------- */
    Timer_Start(TIMER5);

    /* SysTick: 1 kHz (1 ms per tick)
     * SysTick_LOAD = (16 MHz / 1000) - 1 = 15,999
     */
    SysTick_Config(16000 - 1);

    /* ---- System is now running ---------------------------------------- */
    /* All ISRs are active and will set flags as events occur.
     * Main loop will poll these flags and execute application logic.
     */
}

/* ========================================================================
 * System_MainLoop
 *
 * The main event-driven loop.
 * Called from main() after System_Init().
 *
 * Pattern:
 *   while (1) {
 *       System_MainLoop_Iteration();
 *   }
 * ======================================================================== */
void System_MainLoop_Iteration(void)
{
    /* Check for 1ms tick (usually not needed, but available) */
    if (flag_1ms_tick) {
        flag_1ms_tick = 0;
        /* Optional: 1ms tasks */
    }

    /* Generate 10ms and 100ms ticks from SysTick */
    System_Scheduler_Task();

    /* 10ms periodic tasks (motor ramping, button debouncing) */
    if (flag_10ms_tick) {
        flag_10ms_tick = 0;
        Application_10ms_Tick();
    }

    /* 100ms periodic tasks (health checks) */
    if (flag_100ms_tick) {
        flag_100ms_tick = 0;
        Application_100ms_Tick();
    }

    /* 500ms periodic tasks (FSM, Dispatcher, Telemetry) */
    if (flag_500ms_tick) {
        flag_500ms_tick = 0;
        System_Tick_500ms();
    }

    /* Low-priority background tasks (if any) */
    Application_BackgroundTick();
}
