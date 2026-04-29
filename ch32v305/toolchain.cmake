set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR riscv)

set(CMAKE_C_COMPILER    riscv-none-elf-gcc)
set(CMAKE_ASM_COMPILER  riscv-none-elf-gcc)
set(CMAKE_OBJCOPY       riscv-none-elf-objcopy)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_C_FLAGS_INIT "-march=rv32imafc -mabi=ilp32f -msmall-data-limit=8 -mno-save-restore -Os -ffunction-sections -fdata-sections -Wall -g")
set(CMAKE_ASM_FLAGS_INIT "-march=rv32imafc -mabi=ilp32f -g")
set(CMAKE_EXE_LINKER_FLAGS_INIT "-march=rv32imafc -mabi=ilp32f -nostartfiles --specs=nano.specs --specs=nosys.specs -Wl,--gc-sections")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
