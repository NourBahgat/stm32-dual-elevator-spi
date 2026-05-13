#include "IPC.h"
#include "Elevator_FSM.h"
#include "RingBuffer.h"
#include "Usart.h"
#include "Utils.h"
#include "board_config.h"

/* -------------------------------------------------------------------------
 * Private helpers
 * ------------------------------------------------------------------------- */
static const char prv_HEX[] = "0123456789ABCDEF";

/**
 * prv_LogFrame — transmit a labelled hex-dump of one SPI frame.
 * label : short string prefix printed before the dump (e.g. "TX" / "RX")
 * buf   : pointer to SPI_FRAME_SIZE bytes
 */
static void prv_LogFrame(const char *label, const uint8 *buf)
{
    Usart1_TransmitString("[IPC] ");
    Usart1_TransmitString(label);
    Usart1_TransmitString(": ");
    for (uint8 i = 0U; i < SPI_FRAME_SIZE; i++) {
        char s[4];
        s[0] = prv_HEX[(buf[i] >> 4) & 0x0FU];
        s[1] = prv_HEX[ buf[i]       & 0x0FU];
        s[2] = ' ';
        s[3] = '\0';
        Usart1_TransmitString(s);
    }
    Usart1_TransmitString("\r\n");
}

/* -------------------------------------------------------------------------
 * IPC Constants
 * ------------------------------------------------------------------------- */
#define IPC_TIMEOUT_TICKS  5U

/* -------------------------------------------------------------------------
 * IPC State (ISR-protected)
 * ------------------------------------------------------------------------- */
static volatile Ipc_ElevatorState ipc_RemoteState; 

static uint8 ipc_TxBuf[SPI_FRAME_SIZE];
static uint8 ipc_RxBuf[SPI_FRAME_SIZE];

volatile uint8 ipc_TimeoutCounter = 0U;
volatile uint8 Ipc_CommFault = 0U;
volatile uint32 last_spi_rx_timestamp = 0U;  /* defined here to match IPC.h extern */

#if IS_SLAVE()
volatile uint32 ipc_SlaveTimeoutCounter = 0U;
#endif

/* -------------------------------------------------------------------------
 * IPC_Init
 * ------------------------------------------------------------------------- */
void IPC_Init(void)
{
#if IS_MASTER()
  Spi1_Init(SPI_MASTER, SPI_IDLE_LOW, SPI_SAMPLE_FIRST_TRANSITION);
#else
  Spi1_Init(SPI_SLAVE, SPI_IDLE_LOW, SPI_SAMPLE_FIRST_TRANSITION);
#endif

  Spi1_RegisterCallback(IPC_OnFrameReceived);
  ipc_RemoteState.Floor = 1;

  if (IS_SLAVE()) {
    Usart1_TransmitString("[IPC] Arming Slave SPI...\r\n");
    IPC_PackFrame(ipc_TxBuf, (const Ipc_ElevatorState *)&localState);
    if (Spi1_StartTransfer(ipc_TxBuf, ipc_RxBuf) == SPI_OK) {
        Usart1_TransmitString("[IPC] Slave SPI armed and ready.\r\n");
    } else {
        Usart1_TransmitString("[IPC] ERROR: Failed to arm Slave SPI!\r\n");
    }
  }
}

void IPC_GetRemoteStateSnapshot(Ipc_ElevatorState *snapshot)
{
  if (snapshot == (void *)0) return;
  Enter_Critical();
  *snapshot = *((Ipc_ElevatorState *)&ipc_RemoteState);
  Exit_Critical();
}

void IPC_PackFrame(uint8 *TxBuf, const Ipc_ElevatorState *LocalState)
{
  TxBuf[SPI_IDX_HEADER]    = SPI_HEADER_BYTE;
  TxBuf[SPI_IDX_STATE]     = LocalState->State;
  TxBuf[SPI_IDX_FLOOR]     = LocalState->Floor;
  TxBuf[SPI_IDX_DIRECTION] = LocalState->Direction;
  TxBuf[SPI_IDX_QUEUE_HI]  = LocalState->QueueHi;
  TxBuf[SPI_IDX_QUEUE_LO]  = LocalState->QueueLo;
  TxBuf[SPI_IDX_FLAGS]     = LocalState->Flags;

  uint8 chk = 0;
  for (uint8 i = 0; i < SPI_IDX_CHECKSUM; i++) {
    chk ^= TxBuf[i];
  }
  TxBuf[SPI_IDX_CHECKSUM] = chk;
}

uint8 IPC_ValidateFrame(const uint8 *RxBuf)
{
  if (RxBuf[SPI_IDX_HEADER] != SPI_HEADER_BYTE) return SPI_NOK;
  uint8 chk = 0;
  for (uint8 i = 0; i < SPI_IDX_CHECKSUM; i++) {
    chk ^= RxBuf[i];
  }
  return (chk == RxBuf[SPI_IDX_CHECKSUM]) ? SPI_OK : SPI_NOK;
}

void IPC_UnpackFrame(const uint8 *RxBuf, Ipc_ElevatorState *RemoteState)
{
  RemoteState->State     = RxBuf[SPI_IDX_STATE];
  RemoteState->Floor     = RxBuf[SPI_IDX_FLOOR];
  RemoteState->Direction = RxBuf[SPI_IDX_DIRECTION];
  RemoteState->QueueHi    = RxBuf[SPI_IDX_QUEUE_HI];
  RemoteState->QueueLo    = RxBuf[SPI_IDX_QUEUE_LO];
  RemoteState->Flags     = RxBuf[SPI_IDX_FLAGS];
}

void IPC_OnFrameReceived(uint8 *RxBuf)
{
  if (IPC_ValidateFrame(RxBuf) == SPI_OK) {
    IPC_UnpackFrame(RxBuf, (Ipc_ElevatorState *)&ipc_RemoteState);
    ipc_TimeoutCounter = 0U; 
    Ipc_CommFault = 0U;

#if IS_MASTER()
        /* The full-duplex exchange completed successfully, so the Master
         * can drop any pending dispatch request it had encoded in its last
         * TX frame. This prevents the same hall-call from being re-sent on
         * subsequent 50 ms IPC_MasterTick() cycles. */
        prv_LogFrame("MASTER RX (slave state)", RxBuf);
        if (localState.Flags & IPC_FLAG_DISPATCH_REQ) {
            Usart1_TransmitString("[IPC] MASTER: ACK dispatch, clearing DISPATCH_REQ flag\r\n");
            Enter_Critical();
            localState.Flags &= (uint8)(~IPC_FLAG_DISPATCH_REQ);
            Exit_Critical();
        }
        Spi1_DeassertCS();
#endif

#if IS_SLAVE()
    ipc_SlaveTimeoutCounter = 0U;
    prv_LogFrame("SLAVE  RX (master cmd) ", RxBuf);
    if (ipc_RemoteState.Flags & IPC_FLAG_DISPATCH_REQ) {
        uint8 target_floor = IPC_FLAGS_DECODE_FLOOR(ipc_RemoteState.Flags);
        Usart1_TransmitString("[IPC] SLAVE: DISPATCH -> Floor ");
        char fStr[2] = { (char)('0' + target_floor), '\0' };
        Usart1_TransmitString(fStr);
        Usart1_TransmitString("\r\n");
        Elevator_FSM_RequestFloor(target_floor);
    } else {
        Usart1_TransmitString("[IPC] SLAVE: frame OK, no dispatch\r\n");
    }
#endif
  }
  else {
    /* Checksum / header mismatch — log the raw bytes so we can spot shifts */
    const char *who = IS_MASTER() ? "MASTER" : "SLAVE ";
    Usart1_TransmitString("[IPC] ");
    Usart1_TransmitString(who);
    Usart1_TransmitString(": BAD FRAME! ");
    prv_LogFrame("", RxBuf);
    if (IS_MASTER()) Spi1_DeassertCS();
  }

#if IS_SLAVE()
  IPC_PackFrame(ipc_TxBuf, (const Ipc_ElevatorState *)&localState);
  Spi1_StartTransfer(ipc_TxBuf, ipc_RxBuf);
#endif
}

#if IS_MASTER()
void IPC_MasterTick(void)
{
  ipc_TimeoutCounter++;
  if (ipc_TimeoutCounter >= IPC_TIMEOUT_TICKS) {
    Ipc_CommFault = 1U;
  }

  if (Spi1_GetStatus() == SPI_STATUS_IDLE) {
    IPC_PackFrame(ipc_TxBuf, (const Ipc_ElevatorState *)&localState);
    /* Log TX frame; annotate when a dispatch command is embedded */
    if (localState.Flags & IPC_FLAG_DISPATCH_REQ) {
        uint8 f = IPC_FLAGS_DECODE_FLOOR(localState.Flags);
        Usart1_TransmitString("[IPC] MASTER TX: DISPATCH -> Floor ");
        char fStr[2] = { (char)('0' + f), '\0' };
        Usart1_TransmitString(fStr);
        Usart1_TransmitString(" | ");
    }
    prv_LogFrame("MASTER TX", ipc_TxBuf);
    Spi1_AssertCS();
    Spi1_StartTransfer(ipc_TxBuf, ipc_RxBuf);
  }
}
#endif

#if IS_SLAVE()
void IPC_SlaveTick(void)
{
  ipc_SlaveTimeoutCounter++;
  if (ipc_SlaveTimeoutCounter >= (IPC_TIMEOUT_TICKS * 10)) {
    Ipc_CommFault = 1U;
  } else {
    Ipc_CommFault = 0U;
  }
}
#endif