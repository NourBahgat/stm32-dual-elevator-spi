/**
 * Test/master_test.c
 *
 * Driver test firmware for U1 (Master MCU).
 * Build with:  -DDRIVER_TEST_MASTER
 *
 * Phase 1 — USART:
 *   On reset "=== MASTER BOOT ===" appears in the left Virtual Terminal.
 *   The master echoes any character typed back.
 *
 * Phase 2 — SPI:
 *   Every 500 ms the master asserts PA4 (CS) LOW, exchanges one 8-byte
 *   frame with the slave, then de-asserts CS HIGH.
 *   TX / RX hex dumps appear on the Virtual Terminal.
 *
 * Expected output (each 500 ms cycle):
 *   TX: A5 01 01 00 00 00 00 A5
 *   RX OK: A5 02 03 01 00 00 00 A5
 *
 * Proteus wiring:
 *   PB3 SCK   → U2 PB3
 *   PB4 MISO  → U2 PB4    (same pin, no crossing)
 *   PB5 MOSI  → U2 PB5    (same pin, no crossing)
 *   PA4 CS    → U2 PB12   (Master CS out → Slave NSS in)   ← NEW WIRE
 *   PA9  TX   → VT_LEFT RXD
 *   PA10 RX   ← VT_LEFT TXD
 */

#ifdef DRIVER_TEST_MASTER

#include "Nvic.h"
#include "Rcc.h"
#include "Spi.h"
#include "Usart.h"

/* ── Application tick stubs (referenced by System.c's linker closure) ──── */
void Application_10ms_Tick(void)      {}
void Application_100ms_Tick(void)     {}
void Application_500ms_Tick(void)     {}
void Application_BackgroundTick(void) {}

/* ── SysTick (system_time_ms lives in System.c) ─────────────────────────── */
extern volatile uint32 system_time_ms;

static void SysTick_1ms_Start(void)
{
    volatile uint32 *RVR = (volatile uint32 *)0xE000E014U;
    volatile uint32 *CVR = (volatile uint32 *)0xE000E018U;
    volatile uint32 *CSR = (volatile uint32 *)0xE000E010U;
    *RVR = 16000U - 1U;   /* 16 MHz HSI / 1000 = 16 000 ticks per ms */
    *CVR = 0U;
    *CSR = 0x7U;          /* ENABLE | TICKINT | processor clock       */
}

static void Delay_ms(uint32 ms)
{
    uint32 t0 = system_time_ms;
    while ((system_time_ms - t0) < ms) {}
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

/* ── SPI frame buffers (static = ISR-safe) ───────────────────────────────── */
/*
 * TX frame: Header=0xA5 | State=IDLE(1) | Floor=1 | Dir=NONE(0)
 *           QHi=0 | QLo=0 | Flags=0
 *           Checksum = 0xA5^0x01^0x01^0x00^0x00^0x00^0x00 = 0xA5
 */
static uint8 g_MasterTx[SPI_FRAME_SIZE] =
    { 0xA5U, 0x01U, 0x01U, 0x00U, 0x00U, 0x00U, 0x00U, 0xA5U };
static uint8 g_MasterRx[SPI_FRAME_SIZE];

static volatile uint8 g_SpiDone = 0U;

/* ── SPI callback (ISR context) ─────────────────────────────────────────── */
static void OnFrameRx(uint8 *rx)
{
    (void)rx;
    g_SpiDone = 1U;
    /* NOTE: De-assert CS here rather than in the main loop so there is no
     * chance of an extra SCK edge after the last bit.  Spi1_DeassertCS()
     * is a single GPIO write — safe from ISR context. */
    Spi1_DeassertCS();
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
    Usart1_TransmitString("=== MASTER BOOT ===\r\n");
    Usart1_TransmitString("USART OK. Starting SPI test...\r\n");

    /* 4. SPI as Master, Mode 0 (CPOL=0, CPHA=0)
     *    Spi1_Init configures PA4 as CS output (idle HIGH) automatically. */
    Nvic_SetPriority(SPI1_IRQn, 2U);
    Nvic_EnableIrq(SPI1_IRQn);
    Spi1_Init(SPI_MASTER, SPI_IDLE_LOW, SPI_SAMPLE_FIRST_TRANSITION);
    Spi1_RegisterCallback(OnFrameRx);

    /* 5. Event loop */
    while (1)
    {
        uint8  i;
        uint8  chk;

        Delay_ms(500U);

        /* Wait for previous transfer to finish (should be instant) */
        while (Spi1_GetStatus() == SPI_STATUS_BUSY) {}

        PrintFrame("TX: ", g_MasterTx);
        g_SpiDone = 0U;

        /* Assert CS LOW before starting — this enables the slave MISO driver.
         * CS is de-asserted HIGH inside OnFrameRx() once the ISR completes. */
        Spi1_AssertCS();
        Spi1_StartTransfer(g_MasterTx, g_MasterRx);

        /* Spin until ISR fires callback (< 4 us at 21 MHz) */
        while (!g_SpiDone) {}

        /* Validate received frame */
        if (g_MasterRx[SPI_IDX_HEADER] != SPI_HEADER_BYTE) {
            Usart1_TransmitString("RX ERR: bad header\r\n");
            PrintFrame("RAW: ", g_MasterRx);
            continue;
        }
        chk = 0U;
        for (i = 0U; i < SPI_IDX_CHECKSUM; i++) { chk ^= g_MasterRx[i]; }
        if (chk != g_MasterRx[SPI_IDX_CHECKSUM]) {
            Usart1_TransmitString("RX ERR: bad checksum\r\n");
            PrintFrame("RAW: ", g_MasterRx);
            continue;
        }
        PrintFrame("RX OK: ", g_MasterRx);

        /* Echo any typed characters received via USART */
        {
            uint8 ch;
            while (Usart1_GetByte(&ch)) { Usart1_TransmitByte(ch); }
        }
    }

    return 0;
}

#endif /* DRIVER_TEST_MASTER */