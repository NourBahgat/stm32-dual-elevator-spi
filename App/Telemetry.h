/**
 * Telemetry.h
 *
 * Non-blocking, 500ms periodic telemetry system.
 * Collects elevator state every 500ms and sends via UART without blocking.
 *
 * Architecture: Application Layer (sends data), UART Driver (handles Tx)
 * - No blocking delays
 * - No polling loops
 * - DMA runs in background
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "Std_Types.h"

/**
 * Telemetry frame format (Human-readable text):
 *   Dynamic length string, max 40 bytes.
 */
#define TELEMETRY_MAX_LEN      40U

typedef struct {
    char data[TELEMETRY_MAX_LEN];
    uint16 len;
} TelemetryFrame_t;

/**
 * Telemetry_Init
 * Initialize UART driver for non-blocking telemetry transmission.
 */
void Telemetry_Init(void);

/**
 * Telemetry_BuildFrame
 * Populate telemetry frame with current system state.
 * Does not send — just fills the buffer.
 *
 * @param frame Pointer to frame to fill
 * @param fsm_state Current elevator FSM state
 * @param floor Current floor (1-4)
 * @param motor_duty Current motor duty (0-100)
 * @param flags System flags packed into one byte
 */
void Telemetry_BuildFrame(
    TelemetryFrame_t *frame,
    uint8 fsm_state,
    uint8 floor,
    uint8 motor_duty,
    uint8 flags
);

/**
 * Telemetry_SendDMA
 * Non-blocking: start a DMA transfer of the telemetry frame.
 * Returns immediately; DMA works in background.
 *
 * @param frame Pointer to frame to send (must remain valid during transfer)
 * @return 1 if queued, 0 if the transmitter is busy
 */
uint8 Telemetry_SendDMA(const TelemetryFrame_t *frame);

/**
 * @brief  Progress the non-blocking telemetry transmitter.
 *         Call this from the main loop so queued frames are drained one
 *         byte at a time without blocking.
 */
void Telemetry_Tick(void);

/**
 * Telemetry_SendPolling
 * Fallback: transmit using UART polling (TXE polling only, not TC).
 * Used if DMA is unavailable or broken.
 *
 * @param frame Pointer to frame to send
 */
void Telemetry_SendPolling(const TelemetryFrame_t *frame);

#endif /* TELEMETRY_H */
