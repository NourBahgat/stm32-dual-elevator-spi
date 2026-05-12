#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/*
 * board_config.h — Compile-time Master/Slave role selection
 *
 * The project builds TWO separate firmware images from the same codebase:
 *   - Master (Board A): runs Dispatcher + HallCallButtons, drives SPI as master
 *   - Slave  (Board B): responds to SPI, processes dispatch commands from Master
 *
 * HOW TO SELECT THE ROLE:
 *   Define BOARD_ROLE_MASTER or BOARD_ROLE_SLAVE in your CMake / build system:
 *     -DBOARD_ROLE_MASTER=1   (for Master build)
 *     -DBOARD_ROLE_SLAVE=1    (for Slave build)
 *
 *   If neither is defined, default to MASTER for backwards compatibility.
 */

#define ROLE_MASTER  1U
#define ROLE_SLAVE   0U

#if defined(BOARD_ROLE_SLAVE) && BOARD_ROLE_SLAVE == 1
    #define BOARD_ROLE  ROLE_SLAVE
#else
    /* Default to Master if not explicitly set */
    #define BOARD_ROLE  ROLE_MASTER
#endif

/* Convenience macros for conditional compilation */
#define IS_MASTER()  (BOARD_ROLE == ROLE_MASTER)
#define IS_SLAVE()   (BOARD_ROLE == ROLE_SLAVE)

#endif /* BOARD_CONFIG_H */
