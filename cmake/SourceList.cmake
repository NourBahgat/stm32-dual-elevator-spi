file(GLOB_RECURSE SRC_FILES
    ${PROJECT_PATH}/*.c
)

list(FILTER SRC_FILES EXCLUDE REGEX ".*/test_.*\\.c$")
list(FILTER SRC_FILES EXCLUDE REGEX ".*/src/main\\.c$")

if (NOT USE_HAL)
    list(FILTER SRC_FILES EXCLUDE REGEX ".*/(.*(build|STM32-base).*)/.*")
else ()
    list(FILTER SRC_FILES EXCLUDE REGEX ".*/(.*(build).*)|STM32-base/.*")

    file(GLOB_RECURSE SRC_FILES
        ${PROJECT_PATH}/Core/*.c
        ${PROJECT_PATH}/STM32-base-STM32Cube/CMSIS/STM32F4xx/src/*.c
    )

    list(REMOVE_ITEM SRC_FILES ${EXCLUDED_FILES})
endif ()

set(SOURCE_LIST ${SRC_FILES})
