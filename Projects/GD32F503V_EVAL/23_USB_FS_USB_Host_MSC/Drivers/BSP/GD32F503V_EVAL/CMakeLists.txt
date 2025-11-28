project(GD32F503V_EVAL LANGUAGES C CXX ASM)

add_library(GD32F503V_EVAL OBJECT
    # ${DRIVERS_DIR}/BSP/GD32F503V_EVAL/gd32f503v_eval.c
    ${CMAKE_SOURCE_DIR}/Application/Core/Src/gd32f503v_eval.c
    )

target_include_directories(GD32F503V_EVAL PUBLIC
    )

target_link_libraries(GD32F503V_EVAL PUBLIC GD32F50x_standard_peripheral)
