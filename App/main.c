/* When building driver tests define -DDRIVER_TEST_MASTER or
 * -DDRIVER_TEST_SLAVE on the compiler command-line.  Either flag
 * suppresses every symbol in this file so the linker sees only one
 * main() — the one in Test/master_test.c or Test/slave_test.c. */
#if !defined(DRIVER_TEST_MASTER) && !defined(DRIVER_TEST_SLAVE)

/**
 * main.c
 *
 * Dual Elevator STM32F401 Firmware
 * Event-driven, deterministic, DMA-safe architecture
 */

#include "Dispatcher.h"
#include "Elevator_FSM.h"
#include "HallCallButtons.h"
#include "IPC.h"
#include "Pwm.h"
#include "System.h"
#include "Timer.h"
#include "Telemetry.h"
#include "pwm_scheduler.h"
#include "Usart.h"
#include "board_config.h"

/* =========================================================
 * Motor instances
 * ========================================================= */
static PwmContext_t motor_a;
static PwmContext_t motor_b;

/* =========================================================
 * FSM snapshot (single source of truth)
 * ========================================================= */
static Ipc_ElevatorState fsm_snapshot;

/* =========================================================
 * DMA telemetry buffers (double buffer)
 * ========================================================= */
static TelemetryFrame_t tx_frame_a;
static TelemetryFrame_t tx_frame_b;

static TelemetryFrame_t *tx_active = &tx_frame_a;
static TelemetryFrame_t *tx_next   = &tx_frame_b;

static volatile uint8 dma_busy = 0;

/* =========================================================
 * System initialization
 * ========================================================= */
int main(void)
{
    System_Init();

    /* PWM */
    PwmScheduler_Init(&motor_a, TIMER2, PWM_CHANNEL_1);
    PwmScheduler_Init(&motor_b, TIMER2, PWM_CHANNEL_2);

    /* FSM */
    Elevator_FSM_Init(1);

    /* IPC */
    IPC_Init();

#if IS_MASTER()
    Dispatcher_Init();
    HallCallButtons_Init();
#endif

    Telemetry_Init();

    /* ---- Boot Messages ------------------------------------------------- */
#if IS_MASTER()
    Usart1_TransmitString("\r\n[System] Master Elevator (A) Online\r\n");
    Usart1_TransmitString("[System] IPC: SPI Master Mode (62.5kHz)\r\n");
#else
    Usart1_TransmitString("\r\n[System] Slave Elevator (B) Online\r\n");
    Usart1_TransmitString("[System] IPC: SPI Slave Mode (62.5kHz)\r\n");
#endif
    Usart1_TransmitString("[System] Sensors: PB10, PB14, PB12, PB13\r\n\r\n");

    while (1)
    {
        System_MainLoop_Iteration();
    }

    return 0;
}

/* =========================================================
 * 10ms CONTROL LOOP (FSM + PWM)
 * ========================================================= */
static inline void MotorControl_10ms(void)
{
    /* FSM update */
    Elevator_FSM_Tick();

    /* snapshot */
    fsm_snapshot = Elevator_FSM_GetSnapshot();
    fsm_snapshot.MotorDuty = motor_a.current_duty;

    /* motor control */
    PwmScheduler_UpdateState(&motor_a, fsm_snapshot.State);
    PwmScheduler_UpdateState(&motor_b, fsm_snapshot.State);

    PwmScheduler_Tick(&motor_a);
    PwmScheduler_Tick(&motor_b);
}

void Application_10ms_Tick(void)
{
    MotorControl_10ms();
}

/* =========================================================
 * 50ms COORDINATION (Dispatcher + IPC)
 * ========================================================= */
void Application_50ms_Tick(void)
{
#if IS_MASTER()
    Dispatcher_Tick();
    IPC_MasterTick();
#else
    IPC_SlaveTick();
#endif
}

/* =========================================================
 * 100ms RESERVED
 * ========================================================= */
void Application_100ms_Tick(void)
{
#if IS_SLAVE()
    /* Project Requirement: If communication is lost, enter Emergency mode.
     * Ipc_CommFault is set by IPC_SlaveTick after repeated bad frames or timeouts. */
    if (Ipc_CommFault) {
        Elevator_FSM_SetEmergency();
        /* Visual feedback on terminal */
        static uint8 fault_logged = 0;
        if (!fault_logged) {
            Usart1_TransmitString("[System] COMM FAULT detected! Slave entering emergency mode.\r\n");
            fault_logged = 1;
        }
    }
#endif

    /* Print Local Queue frequently (every 100ms) */
    if (IS_MASTER()) {
        Elevator_FSM_PrintQueue((const Ipc_ElevatorState *)&fsm_snapshot, "[Master]");
    } else {
        Elevator_FSM_PrintQueue((const Ipc_ElevatorState *)&fsm_snapshot, "[Slave ]");
    }
}

/* =========================================================
 * 500ms TELEMETRY (DMA safe)
 * ========================================================= */
void Application_500ms_Tick(void)
{
    /* 2. Telemetry (DMA safe) */
    if (dma_busy)
        return;

    Ipc_ElevatorState local_snapshot = fsm_snapshot;

    Telemetry_BuildFrame(tx_next,
                        local_snapshot.State,
                        local_snapshot.Floor,
                        local_snapshot.MotorDuty,
                        local_snapshot.Flags);

    dma_busy = 1;
    Telemetry_SendDMA(tx_next);
}
/* =========================================================
 * DMA complete callback
 * ========================================================= */
void Telemetry_DMA_CompleteCallback(void)
{
    dma_busy = 0;

    /* swap buffers */
    TelemetryFrame_t *tmp = tx_active;
    tx_active = tx_next;
    tx_next = tmp;
}

/* =========================================================
 * Background tasks
 * ========================================================= */
void Application_BackgroundTick(void)
{
#if IS_MASTER()
    HallCallButtons_Process();
#endif
}

#endif /* not DRIVER_TEST_* */

