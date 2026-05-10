/**
 * Telemetry.c
 *
 * Non-blocking telemetry transmission system.
 * Builds 7-byte frame every 500ms and sends via UART (DMA or polling).
 */

#include "Telemetry.h"
#include "Usart.h"
#include "Std_Types.h"

/* ========================================================================
 * Telemetry_Init
 * ======================================================================== */
static TelemetryFrame_t telemetry_active_frame;
static TelemetryFrame_t telemetry_pending_frame;
static uint8 telemetry_active = 0U;
static uint8 telemetry_pending = 0U;
static uint8 telemetry_tx_index = 0U;

void Telemetry_Init(void)
{
    telemetry_active = 0U;
    telemetry_pending = 0U;
    telemetry_tx_index = 0U;
}

/* ========================================================================
 * Telemetry_BuildFrame
 *
 * Populate a telemetry frame with:
 *   SYNC | STATE | FLOOR | DUTY | FLAGS | CHECKSUM
 *
 * Fixed-point, no floating-point needed.
 * Checksum = XOR of (state, floor, duty, flags) for basic error detection.
 * ======================================================================== */
void Telemetry_BuildFrame(
    TelemetryFrame_t *frame,
    uint8 fsm_state,
    uint8 floor,
    uint8 motor_duty,
    uint8 flags
)
{
    if (frame == 0) {
        return;  /* Safety check */
    }

    frame->sync      = 0xAA;           /* Sync byte (sync pattern) */
    frame->fsm_state = fsm_state;      /* FSM state (0-5) */
    frame->floor     = floor;          /* Floor (1-4) */
    frame->motor_duty = motor_duty;    /* Duty (0-100) */
    frame->flags     = flags;          /* Packed system flags */

    /* Checksum = XOR of data bytes (excludes SYNC) */
    frame->checksum  = fsm_state ^ floor ^ motor_duty ^ frame->flags;
}

/* ========================================================================
 * Telemetry_SendDMA
 *
 * Non-blocking transmission queue for UART telemetry.
 *
 * The project does not expose a DMA TX driver in this workspace, so this
 * function behaves like a deterministic software queue:
 *   - copies the frame into internal storage
 *   - returns immediately
 *   - Telemetry_Tick() shifts one byte at a time when TXE is ready
 * ======================================================================== */
uint8 Telemetry_SendDMA(const TelemetryFrame_t *frame)
{
    if (frame == 0) {
        return 0;
    }

    if (telemetry_active != 0U && telemetry_pending != 0U) {
        return 0;
    }

    if (telemetry_active == 0U) {
        telemetry_active_frame = *frame;
        telemetry_active = 1U;
        telemetry_tx_index = 0U;
    } else {
        telemetry_pending_frame = *frame;
        telemetry_pending = 1U;
    }

    return 1;
}

/* ========================================================================
 * Telemetry_SendPolling
 *
 * Compatibility wrapper for the non-blocking queue.
 * ======================================================================== */
void Telemetry_SendPolling(const TelemetryFrame_t *frame)
{
    if (frame == 0) {
        return;
    }

    (void)Telemetry_SendDMA(frame);
}

/* ========================================================================
 * Telemetry_Tick
 *
 * Drives the software transmit queue from the main loop.
 * One byte is attempted per call; if TXE is not ready the function
 * returns immediately, preserving deterministic loop latency.
 * ======================================================================== */
void Telemetry_Tick(void)
{
    uint8 *data;

    if (telemetry_active == 0U) {
        if (telemetry_pending == 0U) {
            return;
        }

        telemetry_active_frame = telemetry_pending_frame;
        telemetry_pending = 0U;
        telemetry_active = 1U;
        telemetry_tx_index = 0U;
    }

    data = (uint8 *)&telemetry_active_frame;

    if (Usart1_TransmitByte(data[telemetry_tx_index]) == Tx_OK) {
        telemetry_tx_index++;

        if (telemetry_tx_index >= TELEMETRY_FRAME_LEN) {
            telemetry_active = 0U;
            telemetry_tx_index = 0U;

            if (telemetry_pending != 0U) {
                telemetry_active_frame = telemetry_pending_frame;
                telemetry_pending = 0U;
                telemetry_active = 1U;
            }
        }
    }
}
