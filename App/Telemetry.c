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
static uint16 prv_AppendString(char *buf, uint16 idx, const char *str) {
    while (*str != '\0') {
        buf[idx++] = *str++;
    }
    return idx;
}

static uint16 prv_AppendUint8(char *buf, uint16 idx, uint8 val) {
    if (val >= 100) {
        buf[idx++] = (char)('0' + (val / 100));
        val %= 100;
        buf[idx++] = (char)('0' + (val / 10));
        val %= 10;
        buf[idx++] = (char)('0' + val);
    } else if (val >= 10) {
        buf[idx++] = (char)('0' + (val / 10));
        val %= 10;
        buf[idx++] = (char)('0' + val);
    } else {
        buf[idx++] = (char)('0' + val);
    }
    return idx;
}

void Telemetry_BuildFrame(
    TelemetryFrame_t *frame,
    uint8 fsm_state,
    uint8 floor,
    uint8 motor_duty,
    uint8 flags
)
{
    uint16 idx = 0;
    if (frame == 0) {
        return;  /* Safety check */
    }

    /* Manually build: [TEL] ST:1 FL:2 PWM:75 FG:0\r\n */
    idx = prv_AppendString(frame->data, idx, "[TEL] ST:");
    idx = prv_AppendUint8(frame->data, idx, fsm_state);
    
    idx = prv_AppendString(frame->data, idx, " FL:");
    idx = prv_AppendUint8(frame->data, idx, floor);
    
    idx = prv_AppendString(frame->data, idx, " PWM:");
    idx = prv_AppendUint8(frame->data, idx, motor_duty);
    
    idx = prv_AppendString(frame->data, idx, " FG:");
    idx = prv_AppendUint8(frame->data, idx, flags);
    
    idx = prv_AppendString(frame->data, idx, "\r\n");

    frame->len = idx;
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
    Usart1_TransmitDMA((uint8 *)frame->data, frame->len);
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
    (void)HAL_UART_Transmit_DMA(&huart1, (uint8_t *)telemetry_active_frame.data, telemetry_active_frame.len);
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

    data = (uint8 *)telemetry_active_frame.data;

    if (Usart1_TransmitByte(data[telemetry_tx_index]) == Tx_OK) {
        telemetry_tx_index++;

        if (telemetry_tx_index >= telemetry_active_frame.len) {
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
            (void)HAL_UART_Transmit_DMA(&huart1, (uint8_t *)telemetry_active_frame.data, telemetry_active_frame.len);
        }
    }
}
#endif
