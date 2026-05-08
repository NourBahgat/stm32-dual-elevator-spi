#include "Ipc.h"
#include "Spi.h"
#include "Elevator_FSM.h"   /* to read local state */

volatile uint8 Ipc_CommFault = 0U;
volatile uint32 last_spi_rx_timestamp = 0U;

static uint8            ipc_TxBuf[SPI_FRAME_SIZE];
static uint8            ipc_RxBuf[SPI_FRAME_SIZE];
static Ipc_ElevatorState ipc_RemoteState;
static volatile uint8   ipc_TimeoutCounter = 0U;
#define IPC_TIMEOUT_TICKS   5U   /* 5 x 50ms = 250ms with no valid frame */

const Ipc_ElevatorState *IPC_GetRemoteState(void) { return &ipc_RemoteState; }

void IPC_Init(void) {
    Spi1_RegisterCallback(IPC_OnFrameReceived);

    /* Slave: pre-arm immediately so it is ready before master's first tick */
    IPC_PackFrame(ipc_TxBuf, &localState);
    Spi1_StartTransfer(ipc_TxBuf, ipc_RxBuf);
}

void IPC_PackFrame(uint8 *TxBuf, const Ipc_ElevatorState *s) {
    TxBuf[SPI_IDX_HEADER]    = SPI_HEADER_BYTE;
    TxBuf[SPI_IDX_STATE]     = s->State;
    TxBuf[SPI_IDX_FLOOR]     = s->Floor;
    TxBuf[SPI_IDX_DIRECTION] = s->Direction;
    TxBuf[SPI_IDX_QUEUE_HI]  = s->QueueHi;
    TxBuf[SPI_IDX_QUEUE_LO]  = s->QueueLo;
    TxBuf[SPI_IDX_FLAGS]     = s->Flags;

    /* Checksum: XOR of bytes 0-6 */
    uint8 cs = 0U;
    for (uint8 i = 0U; i < (SPI_FRAME_SIZE - 1U); i++) { cs ^= TxBuf[i]; }
    TxBuf[SPI_IDX_CHECKSUM] = cs;
}

uint8 IPC_ValidateFrame(const uint8 *RxBuf) {
    if (RxBuf[SPI_IDX_HEADER] != SPI_HEADER_BYTE) return SPI_NOK;
    uint8 cs = 0U;
    for (uint8 i = 0U; i < (SPI_FRAME_SIZE - 1U); i++) { cs ^= RxBuf[i]; }
    return (cs == RxBuf[SPI_IDX_CHECKSUM]) ? SPI_OK : SPI_NOK;
}

void IPC_UnpackFrame(const uint8 *RxBuf, Ipc_ElevatorState *s) {
    s->State     = RxBuf[SPI_IDX_STATE];
    s->Floor     = RxBuf[SPI_IDX_FLOOR];
    s->Direction = RxBuf[SPI_IDX_DIRECTION];
    s->QueueHi   = RxBuf[SPI_IDX_QUEUE_HI];
    s->QueueLo   = RxBuf[SPI_IDX_QUEUE_LO];
    s->Flags     = RxBuf[SPI_IDX_FLAGS];
}

/* Called from SPI ISR — keep it short */
void IPC_OnFrameReceived(uint8 *RxBuf) {
    if (IPC_ValidateFrame(RxBuf) == SPI_OK) {
        IPC_UnpackFrame(RxBuf, &ipc_RemoteState);
        ipc_TimeoutCounter = 0U;   /* reset watchdog */
        Ipc_CommFault      = 0U;
    }
    /* Re-arm slave for the next master cycle */
    IPC_PackFrame(ipc_TxBuf, &localState);
    Spi1_StartTransfer(ipc_TxBuf, ipc_RxBuf);
}

/* Master only: called from 50ms timer ISR */
void IPC_MasterTick(void) {
    ipc_TimeoutCounter++;
    if (ipc_TimeoutCounter >= IPC_TIMEOUT_TICKS) {
        Ipc_CommFault = 1U;   /* FSM and Dispatcher react to this flag */
    }
    IPC_PackFrame(ipc_TxBuf, &localState);
    Spi1_StartTransfer(ipc_TxBuf, ipc_RxBuf);
}