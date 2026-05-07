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
    uint8 Flags;
} Ipc_ElevatorState;

/* Comm fault flag — read by FSM and Dispatcher */
extern volatile uint8 Ipc_CommFault;

void IPC_Init(void);
void IPC_PackFrame  (uint8 *TxBuf, const Ipc_ElevatorState *LocalState);
uint8 IPC_ValidateFrame(const uint8 *RxBuf);
void IPC_UnpackFrame(const uint8 *RxBuf, Ipc_ElevatorState *RemoteState);
void IPC_OnFrameReceived(uint8 *RxBuf);  /* registered as SPI callback */

/* Master only: call from 50ms timer ISR */
void IPC_MasterTick(void);

#endif