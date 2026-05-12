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

#include "IPC.h"
#include "Elevator_FSM.h"   /* volatile Ipc_ElevatorState localState */
#include "Spi.h"
#include "Utils.h"          /* Enter_Critical, Exit_Critical          */
#include "board_config.h"   /* IS_MASTER() / IS_SLAVE() role guards   */

volatile uint8  Ipc_CommFault          = 0U;
volatile uint32 last_spi_rx_timestamp  = 0U;

static uint8             ipc_TxBuf[SPI_FRAME_SIZE];
static uint8             ipc_RxBuf[SPI_FRAME_SIZE];
static Ipc_ElevatorState ipc_RemoteState;
static volatile uint8    ipc_TimeoutCounter = 0U;

#define IPC_TIMEOUT_TICKS  5U   /* 5 × 50 ms = 250 ms with no valid frame */

/* Slave-side watchdog: counts 50 ms ticks between received frames.
 * Only used on ROLE_SLAVE builds — the Master uses ipc_TimeoutCounter
 * driven by IPC_MasterTick().                                         */
#if IS_SLAVE()
static volatile uint8 ipc_SlaveTimeoutCounter = 0U;
#define IPC_SLAVE_TIMEOUT_TICKS  5U   /* 5 × 50 ms = 250 ms              */
#endif

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
    /* Initialise the remote-state buffer to a safe default so the Dispatcher
     * never reads Floor == 0 (below ELEVATOR_MIN_FLOOR) before the first
     * valid SPI frame arrives.                                              */
    ipc_RemoteState.State     = FSM_IDLE;
    ipc_RemoteState.Floor     = ELEVATOR_MIN_FLOOR;  /* 1U — valid ground floor */
    ipc_RemoteState.Direction = DIR_NONE;
    ipc_RemoteState.QueueHi   = 0U;
    ipc_RemoteState.QueueLo   = 0U;
    ipc_RemoteState.Flags     = 0U;

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
 * MASTER path: store validated frame into ipc_RemoteState for Dispatcher.
 * SLAVE  path: store validated frame AND process any dispatch command
 *              encoded in the Flags byte by the Master.
 *
 * CONCURRENCY:
 *   Runs in ISR context. ipc_RemoteState is updated inside a section
 *   that the SPI ISR already owns exclusively, so no extra critical
 *   section is needed here. The main loop always reads ipc_RemoteState
 *   via IPC_GetRemoteStateSnapshot() which uses Enter/Exit_Critical.
 * ------------------------------------------------------------------------- */
void IPC_OnFrameReceived(uint8 *RxBuf)
{
    if (IPC_ValidateFrame(RxBuf) == SPI_OK) {
        IPC_UnpackFrame(RxBuf, &ipc_RemoteState);
        ipc_TimeoutCounter = 0U;   /* reset master-side watchdog */
        Ipc_CommFault      = 0U;

#if IS_SLAVE()
        /* Reset slave-side watchdog — valid frame means Master is alive */
        ipc_SlaveTimeoutCounter = 0U;
        Ipc_CommFault           = 0U;

        /* Issue 7 fix: process any dispatch command the Master encoded
         * in the Flags byte before serialising its own state.
         * The Master writes: Flags = IPC_FLAG_DISPATCH_REQ | encoded_floor.
         * We clear the flag on the ipc_RemoteState copy (not localState)
         * so the Master sees it cleared on the next exchange.            */
        uint8 flags = ipc_RemoteState.Flags;
        if (flags & IPC_FLAG_DISPATCH_REQ) {
            uint8 target_floor = IPC_FLAGS_DECODE_FLOOR(flags);
            Elevator_FSM_RequestFloor(target_floor);
            /* Acknowledge: clear the request bit so Master stops resending */
            ipc_RemoteState.Flags &= (uint8)(~IPC_FLAG_DISPATCH_REQ);
        }
#endif
    }

    /* Re-arm slave TX buffer for the next master cycle.
     * Both roles pre-load their own localState into the TX register so
     * the Slave's state is ready before the Master pulls CS low.        */
    IPC_PackFrame(ipc_TxBuf, (const Ipc_ElevatorState *)&localState);
    Spi1_StartTransfer(ipc_TxBuf, ipc_RxBuf);
}

/* -------------------------------------------------------------------------
 * IPC_MasterTick   (Master only — call every 50 ms from timer)
 * Guarded so it compiles to nothing on Slave builds.
 * ------------------------------------------------------------------------- */
#if IS_MASTER()
void IPC_MasterTick(void)
{
    ipc_TimeoutCounter++;
    if (ipc_TimeoutCounter >= IPC_TIMEOUT_TICKS) {
        Ipc_CommFault = 1U;   /* Dispatcher switches to autonomous mode */
    }
    IPC_PackFrame(ipc_TxBuf, (const Ipc_ElevatorState *)&localState);
    Spi1_StartTransfer(ipc_TxBuf, ipc_RxBuf);
}
#endif /* IS_MASTER */

/* -------------------------------------------------------------------------
 * IPC_SlaveTick   (Slave only — call every 50 ms from timer)
 *
 * Issue 8 fix: gives the Slave its own comm-fault watchdog.
 * If no valid SPI frame arrives within IPC_SLAVE_TIMEOUT_TICKS ticks
 * (250 ms), the Slave sets Ipc_CommFault and enters emergency mode so it
 * stops waiting for Master commands and operates independently.
 * ------------------------------------------------------------------------- */
#if IS_SLAVE()
void IPC_SlaveTick(void)
{
    ipc_SlaveTimeoutCounter++;
    if (ipc_SlaveTimeoutCounter >= IPC_SLAVE_TIMEOUT_TICKS) {
        if (Ipc_CommFault == 0U) {
            /* First detection — latch fault and enter emergency mode */
            Ipc_CommFault = 1U;
            Elevator_FSM_SetEmergency();
        }
        /* Cap counter so it does not overflow on prolonged fault */
        ipc_SlaveTimeoutCounter = IPC_SLAVE_TIMEOUT_TICKS;
    }
}
#endif /* IS_SLAVE */