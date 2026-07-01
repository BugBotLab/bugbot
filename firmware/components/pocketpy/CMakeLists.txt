cmake_minimum_required(VERSION 3.16)

idf_component_register(
    SRCS "pocketpy.c"
    INCLUDE_DIRS "."
    LDFRAGMENTS "pocketpy.lf"
)

# Disable OS (file-system) and POSIX threading — we're on FreeRTOS.
target_compile_definitions(__idf_pocketpy PRIVATE
    PK_ENABLE_OS=0
    PK_ENABLE_THREADS=0
    PK_GC_MIN_THRESHOLD=8000
    PK_VM_STACK_SIZE=2048
)

target_compile_options(__idf_pocketpy PRIVATE
    -Wno-unused-function
    -Wno-unused-variable
    -Wno-implicit-fallthrough
    -Wno-sign-compare
    -Wno-missing-field-initializers
    -Wno-char-subscripts
    -Wno-format
    -Wno-maybe-uninitialized
    -Wno-int-to-pointer-cast
    -Wno-pointer-to-int-cast
    -Wno-empty-body
)
