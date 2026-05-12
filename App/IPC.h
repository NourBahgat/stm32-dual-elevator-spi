#ifndef IPC_H
#define IPC_H

#include "Spi.h"
#include "Std_Types.h"
#include "board_config.h"   /* IS_MASTER() / IS_SLAVE() */

/* ── SPI Frame Layout (8 bytes, fixed-length) ───────────────────────────
 * Byte 0 : Header     = SPI_HEADER_BYTE (sync word)
 * Byte 1 : State      (FSM_IDLE / MOVING_UP / MOVING_DOWN / …)
 * Byte 2 : Floor      (1–4)
 * Byte 3 : Direction  (DIR_NONE / DIR_UP / DIR_DOWN)
 * Byte 4 : QueueHi    (bitmask of floors 3–4)
 * Byte 5 : QueueLo    (bitmask of floors 1–2)
 * Byte 6 : Flags      (command/status byte)
 * Byte 7 : Checksum   (XOR of bytes 0–6)
 * ─────────────────────────────────────────────────────────────────────── */
#define SPI_FRAME_SIZE    8U
#define SPI_HEADER_BYTE   0xA5U

#define SPI_IDX_HEADER    0U
#define SPI_IDX_STATE     1U
#define SPI_IDX_FLOOR     2U
#define SPI_IDX_DIRECTION 3U
#define SPI_IDX_QUEUE_HI  4U
#define SPI_IDX_QUEUE_LO  5U
#define SPI_IDX_FLAGS     6U
#define SPI_IDX_CHECKSUM  7U

#define SPI_OK  0U
#define SPI_NOK 1U

/* The shared state the FSM and Dispatcher read */
typedef struct {
  uint8 State;     /* FSM state enum value   */
  uint8 Floor;     /* Current floor 1-4      */
  uint8 Direction; /* NONE / UP / DOWN       */
  uint8 QueueHi;   /* Request bitmask        */
  uint8 QueueLo;
  uint8 Flags; /* The Master's commands to the Slave*/
  uint8 MotorDuty; /* For telemetry only: Master's motor duty (0-100) */
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
#define IPC_FLAG_DISPATCH_REQ (1U << 2)
#define IPC_FLAG_FLOOR_MASK (0x03U)
#define IPC_FLAGS_ENCODE_FLOOR(f) ((uint8)(((f) - 1U) & IPC_FLAG_FLOOR_MASK))
#define IPC_FLAGS_DECODE_FLOOR(flags)                                          \
  ((uint8)(((flags) & IPC_FLAG_FLOOR_MASK) + 1U))

/* Comm fault flag — read by FSM and Dispatcher */
extern volatile uint8 Ipc_CommFault;
extern volatile uint32 last_spi_rx_timestamp;

void IPC_GetRemoteStateSnapshot(Ipc_ElevatorState *snapshot);

void IPC_Init(void);
void IPC_PackFrame(uint8 *TxBuf, const Ipc_ElevatorState *LocalState);
uint8 IPC_ValidateFrame(const uint8 *RxBuf);
void IPC_UnpackFrame(const uint8 *RxBuf, Ipc_ElevatorState *RemoteState);
void IPC_OnFrameReceived(uint8 *RxBuf); /* registered as SPI callback */

/* Master only: initiate SPI exchange and check for comm fault */
#if IS_MASTER()
void IPC_MasterTick(void);
#endif

/* Slave only: detect Master comm fault and enter emergency mode */
#if IS_SLAVE()
void IPC_SlaveTick(void);
#endif

#endif