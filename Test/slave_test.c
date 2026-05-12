/**
 * Test/slave_test.c
 *
 * Driver test firmware for U2 (Slave MCU).
 * Build with:  -DDRIVER_TEST_SLAVE
 *
 * Phase 1 — USART:
 *   On reset "=== SLAVE BOOT ===" appears in the right Virtual Terminal.
 *
 * Phase 2 — SPI:
 *   The slave pre-arms its TX buffer then waits for the master to assert CS
 *   and clock the frame.  Hardware NSS (PB12) is used — SSM=0 in Spi1_Init
 *   so the MISO output driver is enabled as soon as NSS goes LOW.
 *   On each completed exchange the received frame is printed and the slave
 *   re-arms immediately inside the callback so it is ready for the next cycle.
 *
 * Expected output each 500 ms cycle:
 *   RX OK: A5 01 01 00 00 00 00 A5   (master's frame)
 *
 * Proteus wiring:
 *   PB3  SCK   → U1 PB3
 *   PB4  MISO  → U1 PB4    (same pin, no crossing)
 *   PB5  MOSI  → U1 PB5    (same pin, no crossing)
 *   PB12 NSS   → U1 PA4    (Slave NSS in ← Master CS out)   ← NEW WIRE
 *   PA9  TX    → VT_RIGHT RXD
 *   PA10 RX    ← VT_RIGHT TXD
 *
 * Note on re-arming inside the callback:
 *   The driver's API comment explicitly says the slave should call
 *   Spi1_StartTransfer() again from inside the RX callback.
 *   At that point spi_Status is already IDLE, so StartTransfer succeeds,
 *   pre-loads the TX byte, and re-enables interrupts before returning from
 *   the ISR — leaving the peripheral ready for the next master cycle.
 */

#ifdef DRIVER_TEST_SLAVE

#include "Nvic.h"
#include "Rcc.h"
#include "Spi.h"
#include "Usart.h"

/* ── Application tick stubs ─────────────────────────────────────────────── */
void Application_10ms_Tick(void)      {}
void Application_100ms_Tick(void)     {}
void Application_500ms_Tick(void)     {}
void Application_BackgroundTick(void) {}

/* ── SysTick ─────────────────────────────────────────────────────────────── */
extern volatile uint32 system_time_ms;

static void SysTick_1ms_Start(void)
{
    volatile uint32 *RVR = (volatile uint32 *)0xE000E014U;
    volatile uint32 *CVR = (volatile uint32 *)0xE000E018U;
    volatile uint32 *CSR = (volatile uint32 *)0xE000E010U;
    *RVR = 16000U - 1U;
    *CVR = 0U;
    *CSR = 0x7U;
}

/* ── Hex print helpers ───────────────────────────────────────────────────── */
static const char HEX[] = "0123456789ABCDEF";

static void PrintByte(uint8 b)
{
    char s[3] = { HEX[(b >> 4) & 0xFU], HEX[b & 0xFU], '\0' };
    Usart1_TransmitString(s);
}

static void PrintFrame(const char *label, uint8 *f)
{
    uint8 i;
    Usart1_TransmitString(label);
    for (i = 0U; i < SPI_FRAME_SIZE; i++) {
        PrintByte(f[i]);
        if (i < SPI_FRAME_SIZE - 1U) { Usart1_TransmitString(" "); }
    }
    Usart1_TransmitString("\r\n");
}

/* ── SPI frame buffers ───────────────────────────────────────────────────── */
/*
 * TX frame: Header=0xA5 | State=MOVING_UP(2) | Floor=3 | Dir=UP(1)
 *           QHi=0 | QLo=0 | Flags=0
 *           Checksum = 0xA5^0x02^0x03^0x01^0x00^0x00^0x00
 *                    = 0xA7^0x03^0x01 = 0xA4^0x01 = 0xA5
 */
static uint8 g_SlaveTx[SPI_FRAME_SIZE] =
    { 0xA5U, 0x02U, 0x03U, 0x01U, 0x00U, 0x00U, 0x00U, 0xA5U };
static uint8 g_SlaveRx[SPI_FRAME_SIZE];

/* Set in the ISR callback, consumed in main loop */
static volatile uint8 g_FrameReady = 0U;

/* ── SPI callback (ISR context — keep short) ─────────────────────────────── */
static void OnFrameRx(uint8 *rx)
{
    (void)rx;                           /* g_SlaveRx is already populated    */
    g_FrameReady = 1U;

    /* Re-arm immediately so MISO byte 0 is in the shift register well before
     * the master's next 500 ms cycle begins.
     * The hardware NSS (PB12) going LOW on the next master cycle will
     * re-enable the MISO output driver automatically — no extra action needed. */
    Spi1_StartTransfer(g_SlaveTx, g_SlaveRx);
}

/* ── main ────────────────────────────────────────────────────────────────── */
int main(void)
{
    /* 1. Clocks */
    Rcc_Init();
    Rcc_Enable(RCC_GPIOA);
    Rcc_Enable(RCC_GPIOB);
    Rcc_Enable(RCC_USART1);
    Rcc_Enable(RCC_SPI1);

    /* 2. SysTick */
    SysTick_1ms_Start();

    /* 3. USART */
    Nvic_SetPriority(USART1_IRQn, 3U);
    Usart1_Init();
    Usart1_TransmitString("=== SLAVE BOOT ===\r\n");
    Usart1_TransmitString("USART OK. Arming SPI slave...\r\n");

    /* 4. SPI as Slave, Mode 0 (CPOL=0, CPHA=0).
     *    Spi1_Init detects SLAVE mode and configures:
     *      - PB12 as hardware NSS input (AF5)
     *      - SSM=0 so MISO is driven when NSS is LOW (fixes 0xFF issue) */
    Nvic_SetPriority(SPI1_IRQn, 2U);
    Nvic_EnableIrq(SPI1_IRQn);
    Spi1_Init(SPI_SLAVE, SPI_IDLE_LOW, SPI_SAMPLE_FIRST_TRANSITION);
    Spi1_RegisterCallback(OnFrameRx);

    /* Pre-arm: MISO byte 0 must be in the shift register before the master
     * asserts CS and starts clocking.
     * The callback re-arms after every completed frame. */
    Spi1_StartTransfer(g_SlaveTx, g_SlaveRx);
    Usart1_TransmitString("SPI Slave armed. Waiting for master...\r\n");

    /* 5. Event loop — all real work is done in the ISR callback */
    while (1)
    {
        if (g_FrameReady)
        {
            uint8 i;
            uint8 chk;

            g_FrameReady = 0U;

            /* Validate master's frame */
            if (g_SlaveRx[SPI_IDX_HEADER] != SPI_HEADER_BYTE) {
                Usart1_TransmitString("RX ERR: bad header\r\n");
                PrintFrame("RAW: ", g_SlaveRx);
                continue;
            }
            chk = 0U;
            for (i = 0U; i < SPI_IDX_CHECKSUM; i++) { chk ^= g_SlaveRx[i]; }
            if (chk != g_SlaveRx[SPI_IDX_CHECKSUM]) {
                Usart1_TransmitString("RX ERR: bad checksum\r\n");
                PrintFrame("RAW: ", g_SlaveRx);
                continue;
            }
            PrintFrame("RX OK: ", g_SlaveRx);
        }
    }

    return 0;
}

#endif /* DRIVER_TEST_SLAVE */