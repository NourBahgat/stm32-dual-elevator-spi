#include "pwm_scheduler.h"

/* =========================
 * STATE → SPEED MAPPING
 * ========================= */
static uint8 StateToDuty(ElevatorState_t state)
{
    switch (state)
    {
        case ELEV_MOVING_UP:
        case ELEV_MOVING_DOWN:
            return 100;  /* full speed travel */

        case ELEV_IDLE:
        case ELEV_DOORS_OPEN:
            return 0;

        case ELEV_EMERGENCY:
            return 0;

        default:
            return 0;
    }
}

/* =========================
 * INIT
 * ========================= */
void PwmScheduler_Init(PwmContext_t *ctx, uint8 TimerId, uint8 Channel)
{
    ctx->TimerId = TimerId;
    ctx->Channel = Channel;

    ctx->state = ELEV_IDLE;
    ctx->current_duty = 0;
    ctx->target_duty = 0;
    ctx->emergency_flag = 0;
    ctx->ramp_active = 0;

    Pwm_SetDutyPercent(TimerId, Channel, 0);
}

/* =========================
 * FSM → PWM UPDATE
 * ========================= */
void PwmScheduler_UpdateState(PwmContext_t *ctx, ElevatorState_t state)
{
    ctx->state = state;

    /* Emergency overrides everything */
    if (state == ELEV_EMERGENCY)
    {
        ctx->emergency_flag = 1;
        ctx->target_duty = 0;
        ctx->ramp_active = 1;
        return;
    }

    ctx->emergency_flag = 0;
    ctx->target_duty = StateToDuty(state);

    if (ctx->target_duty != ctx->current_duty)
        ctx->ramp_active = 1;
}

/* =========================
 * RAMP ENGINE (NO BLOCKING)
 * Called from the main loop on the 10ms scheduling flag.
 * ========================= */
void PwmScheduler_Tick(PwmContext_t *ctx)
{
    /* Emergency = hard stop */
    if (ctx->emergency_flag)
    {
        ctx->current_duty = 0;
        Pwm_SetDutyPercent(ctx->TimerId, ctx->Channel, 0);
        return;
    }

    if (!ctx->ramp_active)
        return;

    /* Smooth ramp */
    if (ctx->current_duty < ctx->target_duty)
    {
        ctx->current_duty += 2;  /* step up */
        if (ctx->current_duty >= ctx->target_duty)
        {
            ctx->current_duty = ctx->target_duty;
            ctx->ramp_active = 0;
        }
    }
    else if (ctx->current_duty > ctx->target_duty)
    {
        if (ctx->current_duty >= 2)
            ctx->current_duty -= 2;
        else
            ctx->current_duty = 0;

        if (ctx->current_duty <= ctx->target_duty)
        {
            ctx->current_duty = ctx->target_duty;
            ctx->ramp_active = 0;
        }
    }

    Pwm_SetDutyPercent(ctx->TimerId, ctx->Channel, ctx->current_duty);
}