#include "pwm_scheduler.h"
#include "Usart.h"  /* for debug output */

/* =========================
 * STATE → SPEED MAPPING
 * ========================= */
    static const uint8 state_duty_map[] =
    {
        [ELEV_IDLE]        = 0,
        [ELEV_DOORS_OPEN]  = 20,
        [ELEV_MOVING_UP]   = 100,
        [ELEV_MOVING_DOWN] = 100,
        [ELEV_EMERGENCY]   = 0,
        [ELEV_DOORS_CLOSING] = 0
    };

    static uint8 StateToDuty(ElevatorState_t state)
    {
        if (state >= (sizeof(state_duty_map) / sizeof(state_duty_map[0])))
            return 0;

        return state_duty_map[state];
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
/* =========================================================
 * FSM → PWM UPDATE
 * ========================================================= */
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

    /* Start ramp only if needed */
    if (ctx->target_duty != ctx->current_duty)
        ctx->ramp_active = 1;
    else
        ctx->ramp_active = 0;
}

/* =========================================================
 * RAMP ENGINE (NON-BLOCKING)
 * Called every 10ms from scheduler
 * ========================================================= */
void PwmScheduler_Tick(PwmContext_t *ctx)
{
    /* Emergency handling */
    if (ctx->emergency_flag)
    {
        ctx->current_duty = 0;
        ctx->target_duty = 0;
        ctx->ramp_active = 0;

        Pwm_SetDutyPercent(ctx->TimerId, ctx->Channel, 0);
        return;
    }

    /* Nothing to do */
    if (!ctx->ramp_active)
        return;

    /* Calculate difference — MUST be signed to detect ramp-down correctly.
     * If unsigned: target=0, current=20 → diff = 0xFFEC (underflow!) → ramp
     * engine falsely thinks it needs to go UP and the LED never turns off. */
    sint16 diff = (sint16)ctx->target_duty - (sint16)ctx->current_duty;

    /* Already reached target */
    if (diff == 0)
    {
        ctx->ramp_active = 0;
        return;
    }

    /* Step size: ramp UP */
    if (diff > 0)
    {
        ctx->current_duty += 20U;

        if (ctx->current_duty >= ctx->target_duty)
        {
            ctx->current_duty = ctx->target_duty;
            ctx->ramp_active = 0;
        }
    }
    /* Step size: ramp DOWN */
    else
    {
        if (ctx->current_duty > 20U)
            ctx->current_duty -= 20U;
        else
            ctx->current_duty = 0U;

        if (ctx->current_duty <= ctx->target_duty)
        {
            ctx->current_duty = ctx->target_duty;
            ctx->ramp_active = 0;
        }
    }

    /* Safety clamp */
    if (ctx->current_duty > 100U)
        ctx->current_duty = 100U;

    Pwm_SetDutyPercent(ctx->TimerId, ctx->Channel, ctx->current_duty);
        
    /* DEBUG: Print duty cycle when ramping */
    if (ctx->ramp_active) {
        Usart1_TransmitString("[PWM] Duty: ");
        Usart1_TransmitByte((uint8)('0' + (ctx->current_duty / 10)));
        Usart1_TransmitByte((uint8)('0' + (ctx->current_duty % 10)));
        Usart1_TransmitString("%\r\n");
    }
}