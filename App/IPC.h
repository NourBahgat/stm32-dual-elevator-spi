#ifndef IPC_H
#define IPC_H

#include "Std_Types.h"
#include "Spi.h"

/* The shared state the FSM and Dispatcher read */
typedef struct {
    uint8 State;        /* FSM state enum value   */
    uint8 Floor;        /* Current floor 1-4      */
    uint8 Direction;    /* NONE / UP / DOWN       */
    uint8 QueueHi;      /* Request bitmask        */
    uint8 QueueLo;
    uint8 Flags;        /* The Master's commands to the Slave*/
} Ipc_ElevatorState;

/* ── Flags byte bit definitions (Ipc_ElevatorState.Flags) ─────────────
 *
 *  Bit 2   : IPC_FLAG_DISPATCH_REQ
 *             Master sets this when it wants the Slave to serve a floor.
 *             Slave clears it after calling Elevator_FSM_RequestFloor().
 *
 *  Bits 1:0 : IPC_FLAG_FLOOR_MASK
 *             Encodes the target floor for the Slave: (floor - 1), range 0-3.
 *             Valid only when IPC_FLAG_DISPATCH_REQ is set.
 */
#define IPC_FLAG_DISPATCH_REQ   (1U << 2)
#define IPC_FLAG_FLOOR_MASK     (0x03U)
#define IPC_FLAGS_ENCODE_FLOOR(f)  ((uint8)(((f) - 1U) & IPC_FLAG_FLOOR_MASK))
#define IPC_FLAGS_DECODE_FLOOR(flags) ((uint8)(((flags) & IPC_FLAG_FLOOR_MASK) + 1U))

/* Comm fault flag — read by FSM and Dispatcher */
extern volatile uint8 Ipc_CommFault;
extern volatile uint32 last_spi_rx_timestamp;

void IPC_GetRemoteStateSnapshot(Ipc_ElevatorState *snapshot);

void IPC_Init(void);
void IPC_PackFrame  (uint8 *TxBuf, const Ipc_ElevatorState *LocalState);
uint8 IPC_ValidateFrame(const uint8 *RxBuf);
void IPC_UnpackFrame(const uint8 *RxBuf, Ipc_ElevatorState *RemoteState);
void IPC_OnFrameReceived(uint8 *RxBuf);  /* registered as SPI callback */

/* Master only: call from 50ms timer ISR */
void IPC_MasterTick(void);

#endif