set(CMAKE_SYSTEM_NAME Generic)

set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(TOOLCHAIN_PREFIX                arm-none-eabi-)

find_program(ARM_GCC NAMES ${TOOLCHAIN_PREFIX}gcc)
find_program(ARM_GXX NAMES ${TOOLCHAIN_PREFIX}g++)
find_program(ARM_ASM NAMES ${TOOLCHAIN_PREFIX}gcc)
find_program(ARM_OBJCOPY NAMES ${TOOLCHAIN_PREFIX}objcopy)
find_program(ARM_SIZE NAMES ${TOOLCHAIN_PREFIX}size)
find_program(ARM_LD NAMES ${TOOLCHAIN_PREFIX}ld)

if(NOT ARM_GCC OR NOT ARM_GXX OR NOT ARM_ASM OR NOT ARM_OBJCOPY OR NOT ARM_SIZE OR NOT ARM_LD)
    message(FATAL_ERROR "arm-none-eabi toolchain not found in PATH. Install the GNU Arm Embedded Toolchain or add it to PATH.")
endif()


set(C_FLAGS                         "-fdata-sections -ffunction-sections -fsigned-char -fmessage-length=0 -ffreestanding")
set(CPP_FLAGS                       "-fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_C_FLAGS_INIT "${C_FLAGS}")
set(CMAKE_CXX_FLAGS_INIT "${C_FLAGS} ${CPP_FLAGS}")
set(CMAKE_ASM_FLAGS_INIT "${C_FLAGS}")

# Define compiler settings
set(CMAKE_C_COMPILER    ${ARM_GCC})
set(CMAKE_CXX_COMPILER  ${ARM_GXX})
set(CMAKE_ASM_COMPILER  ${ARM_ASM})

set(CMAKE_OBJCOPY       ${ARM_OBJCOPY})
set(CMAKE_SIZE          ${ARM_SIZE})
set(CMAKE_LINKER        ${ARM_LD})




set(CMAKE_FIND_ROOT_PATH ${BINUTILS_PATH})
set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

