#include "Timer.h"
#include "Timer_Private.h"
#include "Bit_Operations.h"
#include "Nvic.h"

#define NUM_OF_TIMERS 4U

/*
 * Indexed by (TimerId - TIMER2):
 *   [0] = TIM2, [1] = TIM3, [2] = TIM4, [3] = TIM5
 */
static uint32 Timer_BaseAddresses[NUM_OF_TIMERS] = {
    TIM2_BASE_ADDR,
    TIM3_BASE_ADDR,
    TIM4_BASE_ADDR,
    TIM5_BASE_ADDR
};

/*
 * NVIC IRQ numbers for each general-purpose timer (RM0368, Table 38):
 *   TIM2 = 28, TIM3 = 29, TIM4 = 30, TIM5 = 50
 */
static uint8 Timer_NvicIrq[NUM_OF_TIMERS] = {28, 29, 30, 50};

/*
 * User-registered callbacks for async (interrupt-driven) delays.
 * Index matches Timer_BaseAddresses[] order.
 * Initialised to NULL (0); checked before invocation.
 */
static TimerCallback Timer_Callbacks[NUM_OF_TIMERS] = {0};

/*
 * Timer_GetPeripheral
 * -------------------
 * Returns a typed pointer to the TIMx peripheral register block.
 * TimerId is validated by the caller (must be TIMER2–TIMER5).
 */
static TimerType *Timer_GetPeripheral(uint8 TimerId) {
    return (TimerType *) Timer_BaseAddresses[TimerId - TIMER2];
}

/*
 * Timer_Init
 * ----------
 * Configures the time-base of TIMx without starting the counter.
 *
 *   TimerId     : TIMER2 .. TIMER5
 *   Prescaler   : PSC register value.  f_cnt = f_clk / (PSC + 1)
 *   AutoReload  : ARR register value.  Timer overflows at CNT == ARR
 *
 * The EGR_UG (Update Generation) bit is set to immediately transfer
 * PSC and ARR into their shadow registers — without this, the new
 * values only take effect after the first overflow (RM0368 §14.4.6).
 * Setting UG also raises UIF, which is cleared immediately after.
 */
void Timer_Init(uint8 TimerId, uint16 Prescaler, uint16 AutoReload) {
    TimerType *timer = Timer_GetPeripheral(TimerId);

    timer->CR1 = 0;           /* Disable counter; clear all CR1 flags        */
    timer->PSC = Prescaler;   /* Divides input clock: f_cnt = f_clk/(PSC+1)  */
    timer->ARR = AutoReload;  /* Counter resets to 0 after reaching ARR       */
    timer->CNT = 0;           /* Start counting from zero                     */

    SET_BIT(timer->EGR, EGR_UG); /* Force-load PSC & ARR into shadow registers */
    timer->SR = 0;               /* Clear UIF raised by the UG event           */
}

/*
 * Timer_Start
 * -----------
 * Enables the counter by setting CR1_CEN (bit 0 of CR1).
 * The counter begins incrementing from its current CNT value.
 * Call Timer_Init() first to set the time-base.
 */
void Timer_Start(uint8 TimerId) {
    TimerType *tim = Timer_GetPeripheral(TimerId);
    SET_BIT(tim->CR1, CR1_CEN); /* CEN = 1 → counter enabled */
}

/*
 * Timer_Stop
 * ----------
 * Disables the counter by clearing CR1_CEN.
 * CNT is frozen at its current value; it is not reset.
 */
void Timer_Stop(uint8 TimerId) {
    TimerType *tim = Timer_GetPeripheral(TimerId);
    CLEAR_BIT(tim->CR1, CR1_CEN); /* CEN = 0 → counter halted */
}

/*
 * Timer_DelayMs  (blocking / synchronous)
 * ----------------------------------------
 * Blocks the CPU until the requested number of milliseconds elapses.
 *
 * Clock maths (HSI = 16 MHz):
 *   PSC = 15999  →  f_cnt = 16 000 000 / (15999 + 1) = 1000 Hz
 *   Each counter tick = 1 ms.
 *   ARR = DelayMs - 1  →  the counter overflows after exactly DelayMs ticks.
 *
 * One-Pulse Mode (OPM, CR1 bit 3):
 *   The hardware automatically clears CEN after the first overflow,
 *   so the timer stops on its own — no manual stop needed in the loop.
 *
 * Polling:
 *   SR_UIF (bit 0 of SR) is set by hardware on overflow.
 *   We spin here until that flag is set, then clear it.
 *
 * NOTE: PSC = 15999U / 3 (= 5333) was a bug that produced ~3ms ticks.
 *       This is left with a comment for now; correct value is 15999U.
 */
void Timer_DelayMs(uint8 TimerId, uint32 DelayMs) {
    if (DelayMs == 0) return;

    TimerType *timer = (TimerType *)Timer_BaseAddresses[TimerId - TIMER2];

    /* Ensure the counter is enabled */
    SET_BIT(timer->CR1, CR1_CEN);

    for (uint32 i = 0; i < DelayMs; i++) {
        /* Reset counter to ensure a full period wait */
        timer->CNT = 0;
        /* Clear Update Interrupt Flag */
        timer->SR = 0;
        
        /* Busy-wait until the flag is set by hardware on overflow */
        while (!READ_BIT(timer->SR, SR_UIF)) {
            /* CPU is blocked here — no other code runs during this delay */
        }
    }
}

/*
 * Timer_DelayMsAsync  (non-blocking / interrupt-driven)
 * -------------------------------------------------------
 * Starts the timer, registers a callback, and returns immediately.
 * The callback is invoked from the TIMx IRQ handler when the delay expires.
 *
 * Clock maths: same as Timer_DelayMs — see above.
 *
 * Sequence:
 *   1. Store user callback in Timer_Callbacks[].
 *   2. Configure time-base (same PSC/ARR logic as blocking delay).
 *   3. Enable the Update Interrupt in DIER (DIER_UIE, bit 0).
 *   4. Unmask the IRQ line in the NVIC.
 *   5. Start the counter (CR1_CEN).
 *   → On overflow: hardware asserts the IRQ → Timer_HandleIrq() fires.
 */
void Timer_DelayMsAsync(uint8 TimerId, uint32 DelayMs, TimerCallback Callback) {
    TimerType *timer = (TimerType *)Timer_BaseAddresses[TimerId - TIMER2];
    uint8 index  = TimerId - TIMER2;
    uint8 irqNum = Timer_NvicIrq[index];

    Timer_Callbacks[index] = Callback; /* Register callback before IRQ can fire */

    timer->CR1 = 0;                     /* Stop & reset                       */
    timer->PSC = 1599U;                 /* 10 kHz tick rate (100 µs per tick) */
    timer->ARR = (uint16)((DelayMs * 10) - 1); /* Overflow after DelayMs ticks */
    timer->CNT = 0;

    SET_BIT(timer->EGR, EGR_UG); /* Load PSC/ARR into shadow registers        */
    timer->SR = 0;               /* Clear UIF raised by UG                    */

    SET_BIT(timer->CR1, CR1_OPM);  /* One-pulse: auto-stop after overflow     */

    SET_BIT(timer->DIER, DIER_UIE); /* Enable update interrupt in TIMx        */
    Nvic_EnableIrq(irqNum);         /* Unmask the IRQ line in NVIC ISER[]     */

    SET_BIT(timer->CR1, CR1_CEN);  /* Start the counter                       */
}

/*
 * Timer_OcToggleInit
 * ------------------
 * Configures TIMx in Output Compare (OC) Toggle mode on a given channel.
 * The corresponding GPIO pin (configured as AF) toggles every time CNT == CCRx.
 *
 *   Prescaler : PSC value — controls counter tick frequency
 *   Period    : ARR + 1   — counter wraps after 'Period' ticks
 *   CCRx      : fixed at 0, so toggle occurs every ARR+1 ticks
 *               → output period  = 2 × (ARR+1) ticks (toggle = half period)
 *
 * CCMR register layout (RM0368 §14.4.7):
 *   Channels 1 & 2 → CCMR1  |  Channels 3 & 4 → CCMR2
 *   Ch1 & Ch3 occupy bits [7:0]  |  Ch2 & Ch4 occupy bits [15:8]
 *   shift = (Channel - 1) % 2 * 8
 *
 * OCxM[2:0] = 011 → Toggle mode: output flips when CNT matches CCRx
 * Byte written = CCMR_OC_TOGGLE (0x30): OCxM=011, OCxPE=0
 *
 * CCER layout (RM0368 §14.4.9):
 *   Each channel occupies 4 bits: CC1E at bit 0, CC2E at bit 4, etc.
 *   CCxE = 1 enables the output on the pin.
 */
void Timer_OcToggleInit(uint8 TimerId, uint8 Channel, uint16 Prescaler, uint16 Period) {
    TimerType *timer = (TimerType *)Timer_BaseAddresses[TimerId - TIMER2];

    timer->CR1 = 0; /* Stop & reset all CR1 configuration */

    /* Time-base configuration */
    timer->PSC = Prescaler;
    timer->ARR = Period - 1; /* Counter counts 0..ARR then overflows */
    timer->CNT = 0;

    /*
     * Output Compare mode: Toggle (OCxM = 011, byte = 0x30)
     * Clear the full 8-bit OCx field first, then set the toggle mode value.
     * Channel 1 → CCMR1 bits[7:0]    Channel 2 → CCMR1 bits[15:8]
     * Channel 3 → CCMR2 bits[7:0]    Channel 4 → CCMR2 bits[15:8]
     */
    if (Channel <= 2) {
        uint8 shift = (Channel - 1) * 8;
        timer->CCMR1 &= ~((uint32)0xFF << shift);
        timer->CCMR1 |=  ((uint32)CCMR_OC_TOGGLE << shift);
    } else {
        uint8 shift = (Channel - 3) * 8;
        timer->CCMR2 &= ~((uint32)0xFF << shift);
        timer->CCMR2 |=  ((uint32)CCMR_OC_TOGGLE << shift);
    }

    /*
     * CCRx = 0: toggle fires at CNT == 0 (immediately on first tick).
     * &CCR1 + (Channel-1) steps through CCR1, CCR2, CCR3, CCR4 in memory
     * since the struct lays them out contiguously (each 4 bytes apart).
     */
    volatile uint32 *ccr = &timer->CCR1 + (Channel - 1);
    *ccr = 0;

    /*
     * Enable capture/compare output (CCxE bit in CCER).
     * CCER bit layout: CC1E=bit0, CC2E=bit4, CC3E=bit8, CC4E=bit12
     * → bit position = (Channel - 1) * 4
     */
    SET_BIT(timer->CCER, (Channel - 1) * 4);

    /* Load shadow registers and clear any spurious flags before starting */
    SET_BIT(timer->EGR, EGR_UG);
    timer->SR = 0;

    SET_BIT(timer->CR1, CR1_CEN); /* Start the counter */
}

/*
 * Timer_HandleIrq  (internal, not exposed in header)
 * ---------------------------------------------------
 * Shared ISR body called by each TIMx_IRQHandler.
 * 'index' is the array index (0=TIM2, 1=TIM3, 2=TIM4, 3=TIM5).
 *
 * On update event (UIF set):
 *   1. Clear UIF — MUST be done before returning to prevent re-entry.
 *   2. Disable DIER_UIE — prevents further interrupts from this timer.
 *   3. Clear CEN  — redundant in OPM but defensive.
 *   4. Invoke the registered callback if one was set.
 */
static void Timer_HandleIrq(uint8 index) {
    TimerType *timer = (TimerType *)Timer_BaseAddresses[index];

    if (READ_BIT(timer->SR, SR_UIF)) {
        timer->SR = 0;                       /* Clear the update interrupt flag */
        CLEAR_BIT(timer->DIER, DIER_UIE);   /* Disable update interrupt        */
        CLEAR_BIT(timer->CR1,  CR1_CEN);    /* Stop the counter                */

        if (Timer_Callbacks[index] != 0) {
            Timer_Callbacks[index]();        /* Call the user-registered function */
        }
    }
}

/* ISR entry points — names must match the vector table in the startup file */
void TIM2_IRQHandler(void) { Timer_HandleIrq(0); }
void TIM3_IRQHandler(void) { Timer_HandleIrq(1); }
void TIM4_IRQHandler(void) { Timer_HandleIrq(2); }
void TIM5_IRQHandler(void) { Timer_HandleIrq(3); }
