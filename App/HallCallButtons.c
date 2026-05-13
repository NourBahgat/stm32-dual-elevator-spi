/*
 * Each ISR callback ONLY writes into HallCallBuf — no FSM, no Dispatcher.
 * The main loop calls HallCallButtons_Process() to consume the buffer.
 *
 * Pin/line mapping (Port C — PA0/PA1 reserved for TIM2 PWM):
 *   PC0  LINE_0  → Floor 1 UP
 *   PC1  LINE_1  → Floor 2 UP
 *   PC2  LINE_2  → Floor 2 DOWN
 *   PC3  LINE_3  → Floor 3 UP
 *   PC4  LINE_4  → Floor 3 DOWN
 *   PC5  LINE_5  → Floor 4 DOWN
 */

#include "HallCallButtons.h"
#include "Elevator_FSM.h"   /* DIR_UP, DIR_DOWN, ELEVATOR_MIN/MAX_FLOOR */
#include "Dispatcher.h"     /* Dispatcher_AssignElevator, Dispatch_Result */
#include "Gpio.h"
#include "Exti.h"
#include "Usart.h"
#include "Utils.h"          /* Enter_Critical / Exit_Critical             */

/* ── Shared ring buffer ───────────────────────────────────────────── */
HallCallRingBuf HallCallBuf = { .Head = 0U, .Tail = 0U };

/* ── Private: write one entry into the ring buffer (ISR context) ──── */
/*
 * If the buffer is full the call is silently dropped — in a real system
 * you would light an error LED or set a flag.  The buffer size of 8 is
 * intentionally larger than the maximum burst (6 buttons) so a full
 * simultaneous press of all buttons never overflows.
 */
static void buf_push(uint8 floor, uint8 direction)
{
    /*
     * CONCURRENCY: Head is read-modify-written here (3 instructions: not atomic).
     * Two EXTI ISRs at different NVIC priorities can preempt each other and
     * corrupt the index.  Enter_Critical() disables all maskable interrupts
     * for the duration of the index update, making the write atomic.
     */
    Enter_Critical();
    uint8 next_head = (uint8)((HallCallBuf.Head + 1U) & (HALL_CALL_BUF_SIZE - 1U));
    if (next_head != HallCallBuf.Tail) {          /* not full — write entry */
        HallCallBuf.Calls[HallCallBuf.Head].Floor     = floor;
        HallCallBuf.Calls[HallCallBuf.Head].Direction = direction;
        HallCallBuf.Head = next_head;
    }
    /* else: buffer full — silently drop (same policy as before) */
    Exit_Critical();
}

/* ── ISR callbacks (registered with Exti_Init) ────────────────────── */
/* Rules: no FSM calls, no blocking, only buf_push().                  */

static void ISR_Floor1_Up(void)   { buf_push(1U, DIR_UP);   }
static void ISR_Floor2_Up(void)   { buf_push(2U, DIR_UP);   }
static void ISR_Floor2_Down(void) { buf_push(2U, DIR_DOWN);  }
static void ISR_Floor3_Up(void)   { buf_push(3U, DIR_UP);   }
static void ISR_Floor3_Down(void) { buf_push(3U, DIR_DOWN);  }
static void ISR_Floor4_Down(void) { buf_push(4U, DIR_DOWN);  }

/* ── Init ─────────────────────────────────────────────────────────── */

void HallCallButtons_Init(void)
{
    /*
     * Step 1 – configure PC0–PC5 as digital inputs (pull-up).
     */
    Gpio_Init(GPIO_C, 0U, GPIO_INPUT, GPIO_PULL_UP);  /* Floor 1 UP   */
    Gpio_Init(GPIO_C, 1U, GPIO_INPUT, GPIO_PULL_UP);  /* Floor 2 UP   */
    Gpio_Init(GPIO_C, 2U, GPIO_INPUT, GPIO_PULL_UP);  /* Floor 2 DOWN */
    Gpio_Init(GPIO_C, 3U, GPIO_INPUT, GPIO_PULL_UP);  /* Floor 3 UP   */
    Gpio_Init(GPIO_C, 4U, GPIO_INPUT, GPIO_PULL_UP);  /* Floor 3 DOWN */
    Gpio_Init(GPIO_C, 5U, GPIO_INPUT, GPIO_PULL_UP);  /* Floor 4 DOWN */

    /*
     * Step 2 – route each GPIO line through SYSCFG and register callback.
     *           EXTI_EDGE_FALLING: buttons pull line LOW when pressed.
     *           Port source for PC = EXTI_PORT_C (SYSCFG_EXTICR value 0x02).
     */
    Exti_Init(EXTI_LINE_0, EXTI_PORT_C, EXTI_EDGE_FALLING, ISR_Floor1_Up);
    Exti_Init(EXTI_LINE_1, EXTI_PORT_C, EXTI_EDGE_FALLING, ISR_Floor2_Up);
    Exti_Init(EXTI_LINE_2, EXTI_PORT_C, EXTI_EDGE_FALLING, ISR_Floor2_Down);
    Exti_Init(EXTI_LINE_3, EXTI_PORT_C, EXTI_EDGE_FALLING, ISR_Floor3_Up);
    Exti_Init(EXTI_LINE_4, EXTI_PORT_C, EXTI_EDGE_FALLING, ISR_Floor3_Down);
    Exti_Init(EXTI_LINE_5, EXTI_PORT_C, EXTI_EDGE_FALLING, ISR_Floor4_Down);

    /*
     * Step 3 – unmask lines in EXTI->IMR and enable in NVIC.
     *           Lines 0-4 each have their own IRQ vector.
     *           Lines 5-9 share EXTI9_5_IRQHandler (handled by Exti.c).
     */
    Exti_Enable(EXTI_LINE_0);
    Exti_Enable(EXTI_LINE_1);
    Exti_Enable(EXTI_LINE_2);
    Exti_Enable(EXTI_LINE_3);
    Exti_Enable(EXTI_LINE_4);
    Exti_Enable(EXTI_LINE_5);
}

/* ── Main-loop consumer ───────────────────────────────────────────── */

void HallCallButtons_Process(void)
{
    while (HallCallBuf.Tail != HallCallBuf.Head)
    {
        /* Snapshot one entry */
        uint8 floor = HallCallBuf.Calls[HallCallBuf.Tail].Floor;
        uint8 dir   = HallCallBuf.Calls[HallCallBuf.Tail].Direction;

        /* Advance tail before dispatching so the slot is freed even if
         * the dispatcher takes a long time (defensive). */
        HallCallBuf.Tail =
            (uint8)((HallCallBuf.Tail + 1U) & (HALL_CALL_BUF_SIZE - 1U));

        /*
         * Guard: only dispatch once Elevator_FSM_Init() has been called
         * (localState is zero-initialised at link-time, so State == 0 ==
         * FSM_IDLE is safe; a pre-init call simply goes to P3/P4 fallback
         * which is harmless because the queue will absorb it).
         */
        Dispatch_Result winner = Dispatcher_AssignElevator(floor, dir);

        if (winner == DISPATCH_A) {
            Usart1_TransmitString("[Buttons] Call assigned to LOCAL (A)\r\n");
            Elevator_FSM_RequestFloor(floor);
        } else if (winner == DISPATCH_B) {
            Usart1_TransmitString("[Buttons] Call assigned to SLAVE (B)\r\n");
            localState.Flags = (uint8)(IPC_FLAG_DISPATCH_REQ |
                                       IPC_FLAGS_ENCODE_FLOOR(floor));
        } else {
            /* This happens if both elevators are currently moving or in a fault state */
            if (Ipc_CommFault) {
                Usart1_TransmitString("[Buttons] DEFERRED: Slave Offline (SPI Fault)\r\n");
            } else {
                Usart1_TransmitString("[Buttons] DEFERRED: Both elevators busy\r\n");
            }
        }
    }
}