#ifndef PWM_H
#define PWM_H

#include "Std_Types.h"

/* PWM Channel IDs — match TIMx channel numbers (RM0368 section 14) */
#define PWM_CHANNEL_1   1U
#define PWM_CHANNEL_2   2U
#define PWM_CHANNEL_3   3U
#define PWM_CHANNEL_4   4U

/*
 * Pwm_Init
 * --------
 * Configures a timer channel for PWM Mode 1 output.
 * Sets up time-base (PSC, ARR), CCMR mode, CCER output enable, and
 * force-loads shadow registers via EGR_UG. Does NOT start the counter.
 *
 *   TimerId     : TIMER2 .. TIMER5  (from Timer.h)
 *   Channel     : PWM_CHANNEL_1 .. PWM_CHANNEL_4
 *   Prescaler   : PSC register value  ->  f_cnt = f_clk / (PSC + 1)
 *   AutoReload  : ARR register value  ->  PWM period = (ARR + 1) / f_cnt
 *
 * The corresponding GPIO pin must be configured as GPIO_AF with the
 * correct alternate-function number BEFORE this function is called.
 * Call Pwm_Start() after Pwm_Init() to begin the PWM signal.
 */
void Pwm_Init(uint8 TimerId, uint8 Channel, uint16 Prescaler, uint16 AutoReload);

/*
 * Pwm_SetDutyPercent
 * ------------------
 * Updates the PWM duty cycle on a running or stopped channel.
 *
 *   DutyPercent : 0 (always LOW / fan off) .. 100 (always HIGH / full speed)
 *
 * Uses fixed-point arithmetic -- no float.
 *   CCRx = (DutyPercent * ARR) / 100
 *
 * Because output-compare preload (OCxPE) is enabled in Pwm_Init(), the
 * new CCRx value is buffered and applied only at the next timer overflow,
 * preventing glitches mid-period.
 * Values above 100 are clamped to 100 to prevent CCRx exceeding ARR.
 */
void Pwm_SetDutyPercent(uint8 TimerId, uint8 Channel, uint8 DutyPercent);

/*
 * Pwm_Start
 * ---------
 * Enables the channel output (CCxE in CCER) and starts the counter (CR1_CEN).
 * Safe to call after Pwm_Stop() to resume output at the previously set duty.
 */
void Pwm_Start(uint8 TimerId, uint8 Channel);

/*
 * Pwm_Stop
 * --------
 * Disables the channel output (clears CCxE) and stops the counter (clears CR1_CEN).
 * The GPIO pin returns to its idle state; the duty cycle setting is preserved
 * and will resume when Pwm_Start() is called again.
 */
void Pwm_Stop(uint8 TimerId, uint8 Channel);

#endif /* PWM_H */
