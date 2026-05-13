cmake_minimum_required(VERSION 3.30)

project(dual_elevator C)

# ---------------------------------------------------------------------------
# Driver Test Build Options
#
# To build the MASTER test firmware (U1):
#   cmake -DDRIVER_TEST_MASTER=ON ..
#
# To build the SLAVE test firmware (U2):
#   cmake -DDRIVER_TEST_SLAVE=ON ..
#
# To build the production firmware (default):
#   cmake ..
# ---------------------------------------------------------------------------
option(DRIVER_TEST_MASTER "Build master driver test firmware (U1)" OFF)
option(DRIVER_TEST_SLAVE  "Build slave driver test firmware  (U2)" OFF)

if(DRIVER_TEST_MASTER AND DRIVER_TEST_SLAVE)
    message(FATAL_ERROR "Cannot set both DRIVER_TEST_MASTER and DRIVER_TEST_SLAVE at the same time.")
endif()

set(CMAKE_C_STANDARD                11)
set(CMAKE_C_STANDARD_REQUIRED       ON)
set(CMAKE_C_EXTENSIONS              ON)
enable_language(ASM)

set(PROJECT_PATH                         ${CMAKE_CURRENT_SOURCE_DIR})
include(cmake/Device.cmake)
include(cmake/SourceList.cmake)
include(cmake/IncludeList.cmake)
set(CMAKE_OBJCOPY "arm-none-eabi-objcopy")
set(CMAKE_SIZE    "arm-none-eabi-size")

set(LINKER_SCRIPTS_BASE_DIR              ${PROJECT_PATH}/STM32-base/linker/)
set(STARTUP_BASE_DIR                     ${PROJECT_PATH}/STM32-base/startup/)

set(EXECUTABLE                           ${CMAKE_PROJECT_NAME})

# Executable files
add_executable(${EXECUTABLE} ${SOURCE_LIST} ${STARTUP_BASE_DIR}/${SERIES_FOLDER}/${MAPPED_DEVICE}.s)
# Include
target_include_directories(${EXECUTABLE} PUBLIC ${INCLUDE_LIST})

# Project symbols — always inject the device defines, then optionally
# inject the driver-test flag so the preprocessor guards activate.
target_compile_definitions(${EXECUTABLE} PUBLIC ${COMPILER_DEFINE})
target_compile_definitions(${EXECUTABLE} PUBLIC TELEMETRY_USE_REG_DMA)
if(DRIVER_TEST_MASTER)
    target_compile_definitions(${EXECUTABLE} PUBLIC DRIVER_TEST_MASTER)
    target_compile_definitions(${EXECUTABLE} PUBLIC BOARD_ROLE_MASTER=1)
    message(STATUS ">>> Building MASTER driver-test firmware (U1) <<<")
elseif(DRIVER_TEST_SLAVE)
    target_compile_definitions(${EXECUTABLE} PUBLIC DRIVER_TEST_SLAVE)
    target_compile_definitions(${EXECUTABLE} PUBLIC BOARD_ROLE_SLAVE=1)
    message(STATUS ">>> Building SLAVE  driver-test firmware (U2) <<<")
else()
    message(STATUS ">>> Building PRODUCTION firmware <<<")
    if(BOARD_ROLE_SLAVE)
        target_compile_definitions(${EXECUTABLE} PUBLIC BOARD_ROLE_SLAVE=1)
        message(STATUS ">>> Role: SLAVE <<<")
    else()
        target_compile_definitions(${EXECUTABLE} PUBLIC BOARD_ROLE_MASTER=1)
        message(STATUS ">>> Role: MASTER <<<")
    endif()
endif()

# Compiler options
# -ffunction-sections / -fdata-sections are required so that --gc-sections
# (in the linker options below) can discard the unreferenced production
# Application_*_Tick symbols when building test firmwares.
target_compile_options(${EXECUTABLE} PUBLIC
        ${CPU_PARAMETERS}
        -Wall
        -Wpedantic
        -Wno-unused-parameter
        -ffunction-sections
        -fdata-sections
)

# Linker options
target_link_options(${EXECUTABLE} PRIVATE
        ${CPU_PARAMETERS}
        -L${LINKER_SCRIPTS_BASE_DIR}
        -T${LINKER_SCRIPTS_BASE_DIR}/${SERIES_FOLDER}/${DEVICE}.ld
#        -nostartfiles
        -Xlinker --gc-sections
        -Wl,-Map=${CMAKE_PROJECT_NAME}.map
        --specs=nano.specs
        #-u _printf_float                # STDIO float formatting support
)

# Execute post-build to print size and generate HEX/BIN
add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${EXECUTABLE}>
        COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${EXECUTABLE}> ${EXECUTABLE}.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${EXECUTABLE}> ${EXECUTABLE}.bin
        COMMENT "Generating HEX and BIN files..."
)
