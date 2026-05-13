/**
 * Telemetry.c
 *
 * Non-blocking telemetry transmission system.
 * Builds 7-byte frame every 500ms and sends via UART (DMA or polling).
 */

#include "Telemetry.h"
#include "Usart.h"
#include "Std_Types.h"

/* To enable HAL DMA-based telemetry transmission define
 * TELEMETRY_USE_HAL_DMA in your build (e.g. via compiler flags).
 * When enabled the code expects a HAL UART handle `huart1` to be
 * provided by the board/HAL initialization code. If the symbol
 * is not available the build will fail - leave the macro off to
 * use the existing software-driven transmitter.
 */
#ifdef TELEMETRY_USE_HAL_DMA
#include "stm32f4xx_hal.h"
extern UART_HandleTypeDef huart1; /* provided by HAL init code when used */
static volatile uint8 uart_busy = 0U; /* set while DMA transfer in progress */
#endif

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

/* Register-level DMA path: start DMA immediately using Usart1_TransmitDMA.
 * Caller (main) manages `dma_busy` and buffer swapping in the DMA-complete
 * callback `Telemetry_DMA_CompleteCallback()` defined in main.c.
 */
#ifdef TELEMETRY_USE_REG_DMA
    Usart1_TransmitDMA((uint8 *)frame, TELEMETRY_FRAME_LEN);
    return 1;
#endif

#ifdef TELEMETRY_USE_HAL_DMA
    /* DMA path: only one outstanding transfer allowed */
    if (uart_busy != 0U) {
        /* queue the frame if possible */
        if (telemetry_pending != 0U) {
            return 0;
        }

        telemetry_pending_frame = *frame;
        telemetry_pending = 1U;
        return 1;
    }

    /* start DMA immediately */
    telemetry_active_frame = *frame;
    telemetry_active = 1U;
    telemetry_tx_index = 0U;
    uart_busy = 1U;
    (void)HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&telemetry_active_frame, TELEMETRY_FRAME_LEN);
    return 1;
#else
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
#endif
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

#ifdef TELEMETRY_USE_HAL_DMA
    /* When DMA/TX is used the HAL will drive the transfer; nothing to do
     * in the periodic tick unless we want to implement a software timeout
     * or diagnostics. Keep this function a no-op for the DMA path. */
    (void)telemetry_active;
    (void)telemetry_pending;
    (void)telemetry_tx_index;

#elif defined(TELEMETRY_USE_REG_DMA)
    /* Register-level DMA is asynchronous; nothing to do in the periodic tick. */
    (void)telemetry_active;
    (void)telemetry_pending;
    (void)telemetry_tx_index;

#else
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
#endif
}

#ifdef TELEMETRY_USE_HAL_DMA
/* HAL UART Tx complete callback - clears busy flag and starts next
 * queued frame if present. This overrides the weak definition in HAL.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        uart_busy = 0U;

        if (telemetry_pending != 0U) {
            /* move pending -> active and start DMA */
            telemetry_active_frame = telemetry_pending_frame;
            telemetry_pending = 0U;
            telemetry_active = 1U;
            telemetry_tx_index = 0U;
            uart_busy = 1U;
            (void)HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&telemetry_active_frame, TELEMETRY_FRAME_LEN);
        }
    }
}
#endif
