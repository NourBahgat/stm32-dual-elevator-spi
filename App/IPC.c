/*
 * IPC.c — Inter-Processor Communication over SPI
 *
 * Owned by Member 1/2 (SPI driver layer).
 * This file uses the FSM state produced by Member 3 (Elevator_FSM.c).
 *
 * The only change from Member 3's perspective is:
 *   • localState is now 'volatile Ipc_ElevatorState', so every call
 *     that passes &localState to a const Ipc_ElevatorState* must use
 *     an explicit cast to silence the -Wdiscarded-qualifiers warning.
 *   • Elevator_FSM_GetSnapshot() is the preferred way to get an atomic
 *     copy; direct &localState access is used here only in ISR context
 *     where the cast is safe (the ISR cannot be interrupted by itself).
 */

#include "Ipc.h"
#include "Elevator_FSM.h"   /* volatile Ipc_ElevatorState localState */
#include "Spi.h"
#include "Utils.h"          /* Enter_Critical, Exit_Critical          */

volatile uint8  Ipc_CommFault          = 0U;
volatile uint32 last_spi_rx_timestamp  = 0U;

static uint8             ipc_TxBuf[SPI_FRAME_SIZE];
static uint8             ipc_RxBuf[SPI_FRAME_SIZE];
static Ipc_ElevatorState ipc_RemoteState;
static volatile uint8    ipc_TimeoutCounter = 0U;

#define IPC_TIMEOUT_TICKS  5U   /* 5 × 50 ms = 250 ms with no valid frame */

/* -------------------------------------------------------------------------
 * IPC_GetRemoteStateSnapshot
 * Returns an atomic copy of the remote elevator's state.
 * Callers: Dispatcher.c
 * ------------------------------------------------------------------------- */
void IPC_GetRemoteStateSnapshot(Ipc_ElevatorState *snapshot)
{
    Enter_Critical();
    *snapshot = ipc_RemoteState;
    Exit_Critical();
}

/* -------------------------------------------------------------------------
 * IPC_Init
 * Register the SPI receive callback and pre-arm the slave TX buffer.
 * ------------------------------------------------------------------------- */
void IPC_Init(void)
{
    Spi1_RegisterCallback(IPC_OnFrameReceived);

    /* Slave: pre-arm TX buffer so it is ready before the master's first tick.
     * Cast required: localState is volatile; IPC_PackFrame takes const*.
     * This is safe because interrupts have not been enabled yet at this
     * point in the startup sequence.                                        */
    IPC_PackFrame(ipc_TxBuf, (const Ipc_ElevatorState *)&localState);
    Spi1_StartTransfer(ipc_TxBuf, ipc_RxBuf);
}

/* -------------------------------------------------------------------------
 * IPC_PackFrame
 * Serialise an elevator state into the 8-byte SPI frame.
 * ------------------------------------------------------------------------- */
void IPC_PackFrame(uint8 *TxBuf, const Ipc_ElevatorState *s)
{
    TxBuf[SPI_IDX_HEADER]    = SPI_HEADER_BYTE;
    TxBuf[SPI_IDX_STATE]     = s->State;
    TxBuf[SPI_IDX_FLOOR]     = s->Floor;
    TxBuf[SPI_IDX_DIRECTION] = s->Direction;
    TxBuf[SPI_IDX_QUEUE_HI]  = s->QueueHi;
    TxBuf[SPI_IDX_QUEUE_LO]  = s->QueueLo;
    TxBuf[SPI_IDX_FLAGS]     = s->Flags;

    /* Checksum: XOR of bytes 0–6 */
    uint8 cs = 0U;
    for (uint8 i = 0U; i < (SPI_FRAME_SIZE - 1U); i++) {
        cs ^= TxBuf[i];
    }
    TxBuf[SPI_IDX_CHECKSUM] = cs;
}

/* -------------------------------------------------------------------------
 * IPC_ValidateFrame
 * Check header byte and XOR checksum.
 * Returns SPI_OK (0) if valid, SPI_NOK (1) if corrupt.
 * ------------------------------------------------------------------------- */
uint8 IPC_ValidateFrame(const uint8 *RxBuf)
{
    if (RxBuf[SPI_IDX_HEADER] != SPI_HEADER_BYTE) {
        return SPI_NOK;
    }
    uint8 cs = 0U;
    for (uint8 i = 0U; i < (SPI_FRAME_SIZE - 1U); i++) {
        cs ^= RxBuf[i];
    }
    return (cs == RxBuf[SPI_IDX_CHECKSUM]) ? SPI_OK : SPI_NOK;
}

/* -------------------------------------------------------------------------
 * IPC_UnpackFrame
 * Deserialise the 8-byte SPI frame into an elevator state struct.
 * ------------------------------------------------------------------------- */
void IPC_UnpackFrame(const uint8 *RxBuf, Ipc_ElevatorState *s)
{
    s->State     = RxBuf[SPI_IDX_STATE];
    s->Floor     = RxBuf[SPI_IDX_FLOOR];
    s->Direction = RxBuf[SPI_IDX_DIRECTION];
    s->QueueHi   = RxBuf[SPI_IDX_QUEUE_HI];
    s->QueueLo   = RxBuf[SPI_IDX_QUEUE_LO];
    s->Flags     = RxBuf[SPI_IDX_FLAGS];
}

/* -------------------------------------------------------------------------
 * IPC_OnFrameReceived   (SPI callback — called from SPI ISR)
 *
 * CONCURRENCY:
 *   This runs in ISR context. ipc_RemoteState is updated inside
 *   IPC_UnpackFrame while the NVIC has already disabled re-entrant
 *   SPI interrupts, so no extra critical section is needed here.
 *   The main loop always reads ipc_RemoteState via
 *   IPC_GetRemoteStateSnapshot() which uses Enter/Exit_Critical.
 * ------------------------------------------------------------------------- */
void IPC_OnFrameReceived(uint8 *RxBuf)
{
    if (IPC_ValidateFrame(RxBuf) == SPI_OK) {
        IPC_UnpackFrame(RxBuf, &ipc_RemoteState);
        ipc_TimeoutCounter = 0U;   /* reset watchdog */
        Ipc_CommFault      = 0U;
    }
    /* Re-arm slave TX buffer for the next master cycle */
    IPC_PackFrame(ipc_TxBuf, (const Ipc_ElevatorState *)&localState);
    Spi1_StartTransfer(ipc_TxBuf, ipc_RxBuf);
}

/* -------------------------------------------------------------------------
 * IPC_MasterTick   (Master only — call every 50 ms from timer)
 * ------------------------------------------------------------------------- */
void IPC_MasterTick(void)
{
    ipc_TimeoutCounter++;
    if (ipc_TimeoutCounter >= IPC_TIMEOUT_TICKS) {
        Ipc_CommFault = 1U;   /* Dispatcher will switch to autonomous mode */
    }
    IPC_PackFrame(ipc_TxBuf, (const Ipc_ElevatorState *)&localState);
    Spi1_StartTransfer(ipc_TxBuf, ipc_RxBuf);
}