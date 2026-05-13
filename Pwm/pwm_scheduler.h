#ifndef PWM_SCHEDULER_H
#define PWM_SCHEDULER_H

#include "Pwm.h"
#include "Elevator_FSM.h"
#include "Std_Types.h"

/* Elevator states (from FSM) */
typedef enum {
    ELEV_IDLE = FSM_IDLE,
    ELEV_MOVING_UP = FSM_MOVING_UP,
    ELEV_MOVING_DOWN = FSM_MOVING_DOWN,
    ELEV_DOORS_OPEN = FSM_DOORS_OPEN,
    ELEV_EMERGENCY = FSM_EMERGENCY,
    ELEV_DOORS_CLOSING = FSM_DOORS_CLOSING
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