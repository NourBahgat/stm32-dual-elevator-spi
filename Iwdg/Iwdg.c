#include "Iwdg.h"
#include "Utils.h"

/* -------------------------------------------------------------------------
 * IWDG Register Map (Relative to 0x40003000)
 * ------------------------------------------------------------------------- */
#define IWDG_BASE_ADDR  0x40003000U

#define IWDG_KR         REG32(IWDG_BASE_ADDR + 0x00U) /* Key register    */
#define IWDG_PR         REG32(IWDG_BASE_ADDR + 0x04U) /* Prescaler register */
#define IWDG_RLR        REG32(IWDG_BASE_ADDR + 0x08U) /* Reload register    */
#define IWDG_SR         REG32(IWDG_BASE_ADDR + 0x0CU) /* Status register    */

/* -------------------------------------------------------------------------
 * Key Values
 * ------------------------------------------------------------------------- */
#define IWDG_KEY_START   0xCCCCU
#define IWDG_KEY_REFRESH 0xAAAAU
#define IWDG_KEY_ACCESS  0x5555U

/**
 * @brief  Initializes IWDG with a timeout.
 * 
 * Calculations:
 *  - LSI clock is ~32 kHz.
 *  - Using Prescaler = 256 (IWDG_PR = 0x07).
 *  - IWDG clock = 32000 / 256 = 125 Hz (8ms per tick).
 *  - Reload Value = timeout_ms / 8.
 */
void Iwdg_Init(uint32 timeout_ms)
{
    /* 1. Start the watchdog (enables LSI automatically) */
    IWDG_KR = IWDG_KEY_START;

    /* 2. Enable write access to PR and RLR registers */
    IWDG_KR = IWDG_KEY_ACCESS;

    /* 3. Set prescaler to 256 (IWDG_PR = 7)
     *    Divider = 4 * 2^PR = 4 * 2^7 = 4 * 64 = 256? No.
     *    PR[2:0] values: 0:/4, 1:/8, 2:/16, 3:/32, 4:/64, 5:/128, 6:/256. 
     *    So PR = 6 results in /256. */
    IWDG_PR = 0x06U;

    /* 4. Set reload value (RLR is 12-bit, max 4095)
     *    Each tick is 1/125s = 8ms. */
    uint32 rlr = timeout_ms / 8U;
    if (rlr > 0xFFFU) rlr = 0xFFFU;
    IWDG_RLR = rlr;

    /* 5. Wait for registers to be updated (SR flags PVU and RVU) */
    while (IWDG_SR != 0U) {
        /* wait */
    }

    /* 6. Refresh to load the value and keep running */
    IWDG_KR = IWDG_KEY_REFRESH;
}

void Iwdg_Refresh(void)
{
    IWDG_KR = IWDG_KEY_REFRESH;
}
