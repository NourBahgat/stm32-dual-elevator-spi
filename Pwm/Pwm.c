#include "Pwm.h"
#include "Timer_Private.h"   /* TimerType struct, base addresses, CCMR/CCER/EGR bit constants */
#include "Bit_Operations.h"  /* SET_BIT, CLEAR_BIT */
#include "Timer.h"           /* TIMER2..TIMER5, CH1..CH4 */

/*
 * Indexed by (TimerId - TIMER2):
 *   [0] = TIM2, [1] = TIM3, [2] = TIM4, [3] = TIM5
 * Mirrors the layout in Timer.c; re-declared here so Pwm.c is self-contained.
 */
static uint32 Pwm_BaseAddresses[4] = {
    TIM2_BASE_ADDR,
    TIM3_BASE_ADDR,
    TIM4_BASE_ADDR,
    TIM5_BASE_ADDR
};

/*
 * CCR_REG(TIMER, CHANNEL)
 * -----------------------
 * Expands to the lvalue of CCR1, CCR2, CCR3, or CCR4 for a given channel.
 *
 * &TIMER->CCR1 is a pointer to CCR1 (volatile uint32 *).
 * Adding (CHANNEL - 1) steps through contiguous CCR registers in the struct:
 *   +0 -> CCR1,  +1 -> CCR2,  +2 -> CCR3,  +3 -> CCR4
 * Dereferencing gives the register itself as an lvalue for read/write.
 */
#define CCR_REG(TIMER, CHANNEL)  (*((volatile uint32 *)(&(TIMER->CCR1) + (CHANNEL - 1))))

/*
 * Pwm_Init
 * --------
 * Configures TIMx channel N for PWM Mode 1 output.
 * Does NOT start the counter -- call Pwm_Start() afterwards.
 *
 *   TimerId     : TIMER2 .. TIMER5
 *   Channel     : PWM_CHANNEL_1 .. PWM_CHANNEL_4
 *   Prescaler   : PSC value  ->  f_cnt = f_clk / (PSC + 1)
 *   AutoReload  : ARR value  ->  PWM period = (ARR + 1) ticks
 *
 * CCMR register -- PWM Mode 1 + Preload (CCMR_OC_PWM1_PRELOAD = 0x68):
 *   OCxM[2:0] = 110  -> PWM Mode 1: output HIGH while CNT < CCRx, LOW otherwise
 *   OCxPE     = 1    -> Preload enable: CCRx shadow register; new duty only
 *                       takes effect at the next overflow (prevents glitches)
 *
 * CCMR register layout (RM0368 section 14.4.7):
 *   Channels 1 & 2 -> CCMR1  |  Channels 3 & 4 -> CCMR2
 *   Ch1 & Ch3 -> bits [7:0]  |  Ch2 & Ch4 -> bits [15:8]
 *   shift = (Channel - 1) % 2 * 8
 *
 * CCER layout (RM0368 section 14.4.9):
 *   CCxE bit enables the channel output on the pin.
 *   CC1E = bit 0, CC2E = bit 4, CC3E = bit 8, CC4E = bit 12
 *   -> position = (Channel - 1) * 4
 *
 * CR1_ARPE (Auto-Reload Preload Enable, bit 7):
 *   Ensures ARR is also buffered in a shadow register -- new ARR values
 *   only apply after the current period ends, preventing partial periods.
 *
 * EGR_UG (Update Generation, bit 0 of EGR):
 *   Forces an immediate update event to transfer PSC, ARR, and CCRx
 *   from their preload registers into the active (shadow) registers.
 *   Without this, the first PWM period may use stale values.
 *   Setting UG also raises UIF in SR, which is cleared immediately after.
 */
void Pwm_Init(uint8 TimerId, uint8 Channel, uint16 Prescaler, uint16 AutoReload) {
    TimerType *timer = (TimerType *)Pwm_BaseAddresses[TimerId - TIMER2];

    /* Time-base configuration */
    timer->CR1 = 0;          /* Stop counter; clear all CR1 flags             */
    timer->PSC = Prescaler;  /* f_cnt = f_clk / (PSC + 1)                    */
    timer->ARR = AutoReload; /* Period = ARR + 1 ticks                        */
    timer->CNT = 0;          /* Reset counter to zero                         */

    /*
     * PWM Mode 1 + output-compare preload on selected channel.
     * Byte written = CCMR_OC_PWM1_PRELOAD (0x68):
     *   bits [6:4] OCxM = 110  -> PWM Mode 1
     *   bit  [3]   OCxPE = 1   -> preload enable
     */
    if (Channel <= 2) {
        uint8 shift = (Channel - 1) * 8; /* Ch1 -> shift=0,  Ch2 -> shift=8  */
        timer->CCMR1 &= ~((uint32)0xFF << shift);                  /* Clear OCx field  */
        timer->CCMR1 |=  ((uint32)CCMR_OC_PWM1_PRELOAD << shift); /* Set PWM mode     */
    } else {
        uint8 shift = (Channel - 3) * 8; /* Ch3 -> shift=0,  Ch4 -> shift=8  */
        timer->CCMR2 &= ~((uint32)0xFF << shift);
        timer->CCMR2 |=  ((uint32)CCMR_OC_PWM1_PRELOAD << shift);
    }

    /*
     * Enable the channel output on the GPIO pin.
     * CCxE is at bit (Channel-1)*4 in CCER.
     */
    SET_BIT(timer->CCER, (Channel - 1) * 4);

    /* Initialise duty cycle to 0% -- output stays LOW until Pwm_SetDutyPercent() */
    CCR_REG(timer, Channel) = 0;

    /* Enable auto-reload preload so ARR changes take effect at next update */
    SET_BIT(timer->CR1, CR1_ARPE);

    /* Force-load all shadow registers, then clear the resulting UIF flag */
    SET_BIT(timer->EGR, EGR_UG);
    timer->SR = 0;
}

/*
 * Pwm_SetDutyPercent
 * ------------------
 * Adjusts the PWM duty cycle without restarting the timer.
 *
 *   DutyPercent : 0 = always LOW (fan off),  100 = always HIGH (full speed)
 *
 * Fixed-point formula (no floating-point required):
 *   CCRx = (DutyPercent * ARR) / 100
 *
 * Because OCxPE = 1 (preload enabled in Pwm_Init), writing CCRx updates
 * the shadow register only; the new duty cycle takes effect at the next
 * timer overflow -- this prevents mid-period glitches on the PWM output.
 *
 * uint32 intermediate prevents 16-bit overflow:
 *   max = 100 * 65535 = 6 553 500 -- safely within uint32 range.
 *
 * Clamped to 100 to prevent CCRx > ARR (which would lock output HIGH).
 */
void Pwm_SetDutyPercent(uint8 TimerId, uint8 Channel, uint8 DutyPercent) {
    TimerType *timer = (TimerType *)Pwm_BaseAddresses[TimerId - TIMER2];

    /* Clamp input -- values > 100 are invalid and would exceed ARR */
    if (DutyPercent > 100) {
        DutyPercent = 100;
    }

    uint32 arr = timer->ARR;                           /* Read active period value  */
    uint32 ccr = ((uint32)DutyPercent * arr) / 100UL; /* Scale duty to timer ticks */

    CCR_REG(timer, Channel) = ccr; /* Write to CCRx shadow register */
}

/*
 * Pwm_Start
 * ---------
 * Enables the PWM output and starts the counter.
 *
 * Sequence:
 *   1. Confirm CCxE is set in CCER (channel output enabled on pin).
 *   2. Set CR1_CEN to start the counter incrementing.
 *
 * Pwm_Init() already sets CCxE, but re-setting it here is defensive --
 * it handles the case where Pwm_Stop() disabled the channel without
 * stopping the counter, and the user calls Pwm_Start() again.
 */
void Pwm_Start(uint8 TimerId, uint8 Channel) {
    TimerType *tim = (TimerType *)Pwm_BaseAddresses[TimerId - TIMER2];

    SET_BIT(tim->CCER, (Channel - 1) * 4); /* Re-enable channel output (CCxE) */
    SET_BIT(tim->CR1,  CR1_CEN);           /* Start the counter               */
}

/*
 * Pwm_Stop
 * --------
 * Disables the PWM output and stops the counter.
 *
 * Clearing CCxE disconnects the timer from the GPIO pin -- the pin
 * returns to its idle state (determined by GPIO OTYPER/PUPDR config).
 * Clearing CR1_CEN freezes the counter at its current value.
 */
void Pwm_Stop(uint8 TimerId, uint8 Channel) {
    TimerType *tim = (TimerType *)Pwm_BaseAddresses[TimerId - TIMER2];

    CLEAR_BIT(tim->CCER, (Channel - 1) * 4); /* Disable channel output (CCxE) */
    CLEAR_BIT(tim->CR1,  CR1_CEN);           /* Stop the counter              */
}
