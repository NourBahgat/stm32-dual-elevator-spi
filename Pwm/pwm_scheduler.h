#ifndef PWM_SCHEDULER_H
#define PWM_SCHEDULER_H

#include <stdint.h>
#include "Pwm.h"

/* Elevator states (from FSM) */
typedef enum {
    ELEV_IDLE = 0,
    ELEV_MOVING_UP,
    ELEV_MOVING_DOWN,
    ELEV_DOORS_OPEN,
    ELEV_EMERGENCY
} ElevatorState_t;

/* Motor control context */
typedef struct {
    uint8 TimerId;
    uint8 Channel;

    volatile ElevatorState_t state;
    volatile uint8 current_duty;
    volatile uint8 target_duty;

    volatile uint8 emergency_flag;
    volatile uint8 ramp_active;
} PwmContext_t;

/* API */
void PwmScheduler_Init(PwmContext_t *ctx, uint8 TimerId, uint8 Channel);
void PwmScheduler_UpdateState(PwmContext_t *ctx, ElevatorState_t state);
void PwmScheduler_Tick(PwmContext_t *ctx);  /* called from the 10ms main-loop tick */

#endif