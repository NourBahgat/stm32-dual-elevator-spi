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
#include "Bit_Operations.h"
#include "Exti.h"
#include "Gpio.h"
#include "Nvic.h"
#include "Pwm.h"
#include "pwm_scheduler.h"
#include "Rcc.h"
#include "Timer.h"
#include "Timer_Private.h"   /* TimerType, TIM5_BASE_ADDR, SR_UIF */
#include "Usart.h"
#include "Utils.h"           /* REG32, Enter_Critical, Exit_Critical */
#include "../Dma/Dma.h"

/* TIM5 register access via base address */
#define TIM5  ((TimerType *)TIM5_BASE_ADDR)

/* -------------------------------------------------------------------------
 * SysTick_Config_Local
 *
 * Configures the ARM SysTick timer directly via its memory-mapped registers.
 * The CMSIS SysTick_Config() requires an NVIC header that conflicts with this
 * project's custom NVIC driver, so we configure it manually here.
 *
 * Registers (ARM Cortex-M4 TRM):
 *   0xE000E014  SYST_RVR  — Reload Value Register   (ticks per period)
 *   0xE000E018  SYST_CVR  — Current Value Register  (reset to 0)
 *   0xE000E010  SYST_CSR  — Control/Status Register
 *     bit 0 ENABLE   = 1  (counter running)
 *     bit 1 TICKINT  = 1  (generate SysTick exception)
 *     bit 2 CLKSOURCE= 1  (processor clock, not external)
 *     → write 0x7
 * ------------------------------------------------------------------------- */
static void SysTick_Config_Local(uint32 ticks)
{
    volatile uint32 *SYST_RVR = (volatile uint32 *)0xE000E014U;
    volatile uint32 *SYST_CVR = (volatile uint32 *)0xE000E018U;
    volatile uint32 *SYST_CSR = (volatile uint32 *)0xE000E010U;

    *SYST_RVR = (ticks - 1U);   /* reload value        */
    *SYST_CVR = 0U;              /* clear current value */
    *SYST_CSR = 0x7U;            /* enable, use processor clock, generate IRQ */
}

/* ========================================================================
 * Global Time Counter (SysTick ISR increments this)
 * ======================================================================== */
volatile uint32 system_time_ms = 0;

/* ========================================================================
 * Event Flags (ISRs set these, application clears)
 * ======================================================================== */
volatile uint8 flag_1ms_tick   = 0;
volatile uint8 flag_50ms_tick  = 0;
volatile uint8 flag_10ms_tick  = 0;
volatile uint8 flag_100ms_tick = 0;
volatile uint8 flag_500ms_tick = 0;

/* ========================================================================
 * Scheduler State (main loop tracking)
 * Used to generate 50ms, 10ms and 100ms ticks from 1ms SysTick
 * ======================================================================== */
static uint32 last_50ms_tick  = 0;
static uint32 last_10ms_tick  = 0;
static uint32 last_100ms_tick = 0;
/* PWM contexts are owned by main.c to avoid duplicate instances. */
/* ========================================================================
 * SysTick_Handler
 *
 * Called every 1 ms by the hardware timer.
 * CRITICAL: Keep this under 2 µs execution time.
 * ======================================================================== */
void SysTick_Handler(void) {
    system_time_ms++;
    flag_1ms_tick = 1;
}

/* ========================================================================
 * TIM5_IRQHandler
 *
 * Called every 500 ms by TIM5.
 * CRITICAL: Keep this under 2 µs execution time.
 * ======================================================================== */
void TIM5_IRQHandler(void) {
    /* Check update interrupt flag (UIF, bit 0 of SR) */
    if (READ_BIT(TIM5->SR, SR_UIF)) {
        // TIM5->SR = 0;        /* Clear UIF                          */
        CLEAR_BIT(TIM5->SR, SR_UIF);
        flag_500ms_tick = 1; /* Signal application layer           */
    }
}

/* ========================================================================
 * System_Scheduler_Task (private)
 * ======================================================================== */
static void System_Scheduler_Task(void) {
    if (system_time_ms - last_50ms_tick >= 50U) {
        last_50ms_tick = system_time_ms;
        flag_50ms_tick = 1;
    }
    if (system_time_ms - last_10ms_tick >= 10U) {
        last_10ms_tick = system_time_ms;
        flag_10ms_tick = 1;
    }
    if (system_time_ms - last_100ms_tick >= 100U) {
        last_100ms_tick = system_time_ms;
        flag_100ms_tick = 1;
    }
}

/* ========================================================================
 * System_Tick_500ms
 * ======================================================================== */
void System_Tick_500ms(void) { Application_500ms_Tick(); }

/* ========================================================================
 * System_Init
 *
 * One-time initialization of all subsystems.
 * Called from main() before entering the event loop.
 *
 * Order:
 *   1. Clock tree  (enables peripherals)
 *   2. NVIC priorities
 *   3. Peripheral drivers (UART, Timer, PWM)
 *   4. Enable IRQs
 *   5. Start timers / SysTick
 * ======================================================================== */
void System_Init(void) {

    /* ---- 1. Enable Peripheral Clocks (CRITICAL) ----------------------- */
    Rcc_Init();
    Rcc_Enable(RCC_GPIOA);
    Rcc_Enable(RCC_GPIOB);
    Rcc_Enable(RCC_GPIOC);
    Rcc_Enable(RCC_USART1);
    Rcc_Enable(RCC_SPI1);
    Rcc_Enable(RCC_TIM2);
    Rcc_Enable(RCC_TIM5);
    Rcc_Enable(RCC_SYSCFG);
    Rcc_Enable(RCC_DMA2);

    /* ---- 2. NVIC Priority Configuration ------------------------------- */
    /* SysTick has a fixed priority of 15 in ARM Cortex-M.
     * We only configure the priorities we explicitly need: */
    Nvic_SetPriority(TIM5_IRQn,   2U);  /* 500 ms scheduler     */
    Nvic_SetPriority(USART1_IRQn, 3U);  /* UART telemetry RX    */
    /* Emergency Stop (EXTI15) priority = 0 is set in Elevator_FSM_Init() */

    /* ---- 3. Driver Initialization ------------------------------------- */

    /* UART for telemetry (sets up PA9/PA10 as TX/RX AF7, enables RXNEIE) */
    Usart1_Init();

    /* DMA subsystem for USART1 TX (register-level) */
    Dma_Init();

    /* TIM5: 500 ms period
     *   PSC = 15999 → f_cnt = 16 MHz / 16000 = 1 kHz
     *   ARR = 499   → overflow every 500 ticks = 500 ms */
    Timer_Init(TIMER5, 15999U, 499U);

    /* PWM on TIM2 for motor LED simulation (10 kHz)
     *   PSC = 15  → f_cnt = 16 MHz / 16 = 1 MHz
     *   ARR = 99  → PWM period = 100 µs = 10 kHz */
    Gpio_Init(GPIO_A, 0U, GPIO_AF, GPIO_PUSH_PULL);  /* PA0 = TIM2_CH1 */
    Gpio_SetAF(GPIO_A, 0U, GPIO_AF1);
    Gpio_Init(GPIO_A, 1U, GPIO_AF, GPIO_PUSH_PULL);  /* PA1 = TIM2_CH2 */
    Gpio_SetAF(GPIO_A, 1U, GPIO_AF1);

    Pwm_Init(TIMER2, PWM_CHANNEL_1, 15U, 99U);  /* Motor A */
    Pwm_Init(TIMER2, PWM_CHANNEL_2, 15U, 99U);  /* Motor B */
    Pwm_Start(TIMER2, PWM_CHANNEL_1);
    Pwm_Start(TIMER2, PWM_CHANNEL_2);

    /* ---- 4. Enable IRQs ----------------------------------------------- */
    Nvic_SetPriority(SPI1_IRQn,   2U);
    Nvic_EnableIrq(TIM5_IRQn);
    Nvic_EnableIrq(SPI1_IRQn);

    /* ---- 5. Start Timers ---------------------------------------------- */
    Timer_Start(TIMER5);

    /* SysTick: 1 kHz → interrupt every 1 ms
     *   LOAD = 16 MHz / 1000 = 16000 counts */
    SysTick_Config_Local(16000U);
}

/* ========================================================================
 * System_MainLoop_Iteration
 *
 * Single iteration of the event loop. Called from while(1) in main().
 * ======================================================================== */
void System_MainLoop_Iteration(void) {

    /* 1 ms tick — available for fine-grained tasks if needed */
    if (flag_1ms_tick) {
        flag_1ms_tick = 0;
    }

    /* Generate derived 10 ms and 100 ms ticks */
    System_Scheduler_Task();

    /* 10 ms: motor PWM ramping */
    if (flag_10ms_tick) {
        flag_10ms_tick = 0;
        Application_10ms_Tick();
    }

    /* 100 ms: comms health check */
    if (flag_100ms_tick) {
        flag_100ms_tick = 0;
        Application_100ms_Tick();
    }

    /* 50 ms: dispatcher + IPC sync */
    if (flag_50ms_tick) {
        flag_50ms_tick = 0;
        Application_50ms_Tick();
    }

    /* 500 ms: telemetry */
    if (flag_500ms_tick) {
        flag_500ms_tick = 0;
        System_Tick_500ms();
    }
  

    /* Background: low-priority tasks (UART drain, button debounce) */
    Application_BackgroundTick();
}
