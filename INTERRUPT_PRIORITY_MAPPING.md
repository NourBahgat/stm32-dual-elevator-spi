# Interrupt Priority Mapping

This document outlines the NVIC (Nested Vectored Interrupt Controller) priority configuration for the Dual Elevator System.

## Priority Levels

In the ARM Cortex-M architecture, lower numerical values represent higher logical priorities.

| Interrupt Source | IRQ Number | Priority Level | Description |
| :--- | :--- | :--- | :--- |
| **Emergency Stop** | 40 (EXTI15_10) | **0** | Highest priority. Ensures immediate FSM override for safety. |
| **IPC (SPI1)** | 35 (SPI1) | **2** | High priority. Critical for real-time synchronization between Master and Slave. |
| **Scheduler (TIM5)** | 50 (TIM5) | **2** | High priority. Drives the 500ms application tasks. |
| **Telemetry (USART1)** | 37 (USART1) | **3** | Medium priority. Handles telemetry data reception. |
| **Telemetry DMA (DMA2)** | 70 (Stream 7) | **4** | Lower priority. Signals completion of non-blocking telemetry transmissions. |
| **System Tick** | - | **15** | Default lowest priority. Drives the 1ms system time. |

## Design Rationale

1.  **Safety First (Priority 0):** The Emergency Stop must be able to preempt any other task, including active communication or motor control, to ensure passenger safety.
2.  **Real-Time Sync (Priority 2):** SPI communication and the main system scheduler share the same high priority level to ensure the distributed control loop remains tightly synchronized without being delayed by standard I/O.
3.  **Non-Blocking I/O (Priority 3-4):** Telemetry tasks are assigned lower priorities as they are diagnostic in nature and should not interfere with the core elevator control logic.
4.  **DMA completion (Priority 4):** Since the DMA handles the heavy lifting of data movement, its interrupt only signals a finished task, making it suitable for a lower priority.
