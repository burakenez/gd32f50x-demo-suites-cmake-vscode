project(GD32F50x_standard_peripheral LANGUAGES C CXX ASM)

# Comment-out unused source files.
add_library(GD32F50x_standard_peripheral OBJECT
	${DRIVERS_DIR}/GD32F50x_standard_peripheral/Source/gd32f50x_adc.c
    ${DRIVERS_DIR}/GD32F50x_standard_peripheral/Source/gd32f50x_bkp.c
    ${DRIVERS_DIR}/GD32F50x_standard_peripheral/Source/gd32f50x_can.c
    ${DRIVERS_DIR}/GD32F50x_standard_peripheral/Source/gd32f50x_cau.c
    ${DRIVERS_DIR}/GD32F50x_standard_peripheral/Source/gd32f50x_cmp.c
    ${DRIVERS_DIR}/GD32F50x_standard_peripheral/Source/gd32f50x_crc.c
    ${DRIVERS_DIR}/GD32F50x_standard_peripheral/Source/gd32f50x_ctc.c
    ${DRIVERS_DIR}/GD32F50x_standard_peripheral/Source/gd32f50x_dac.c
    ${DRIVERS_DIR}/GD32F50x_standard_peripheral/Source/gd32f50x_dbg.c
    ${DRIVERS_DIR}/GD32F50x_standard_peripheral/Source/gd32f50x_dma.c
    ${DRIVERS_DIR}/GD32F50x_standard_peripheral/Source/gd32f50x_exmc.c
    ${DRIVERS_DIR}/GD32F50x_standard_peripheral/Source/gd32f50x_exti.c
    ${DRIVERS_DIR}/GD32F50x_standard_peripheral/Source/gd32f50x_fmc.c
    ${DRIVERS_DIR}/GD32F50x_standard_peripheral/Source/gd32f50x_fwdgt.c
    ${DRIVERS_DIR}/GD32F50x_standard_peripheral/Source/gd32f50x_gpio.c
    ${DRIVERS_DIR}/GD32F50x_standard_peripheral/Source/gd32f50x_hau.c
    ${DRIVERS_DIR}/GD32F50x_standard_peripheral/Source/gd32f50x_i2c.c
    ${DRIVERS_DIR}/GD32F50x_standard_peripheral/Source/gd32f50x_misc.c
    ${DRIVERS_DIR}/GD32F50x_standard_peripheral/Source/gd32f50x_pmu.c
    ${DRIVERS_DIR}/GD32F50x_standard_peripheral/Source/gd32f50x_rcu.c
    ${DRIVERS_DIR}/GD32F50x_standard_peripheral/Source/gd32f50x_rtc.c
    ${DRIVERS_DIR}/GD32F50x_standard_peripheral/Source/gd32f50x_spi.c
    ${DRIVERS_DIR}/GD32F50x_standard_peripheral/Source/gd32f50x_syscfg.c
    ${DRIVERS_DIR}/GD32F50x_standard_peripheral/Source/gd32f50x_timer.c
    ${DRIVERS_DIR}/GD32F50x_standard_peripheral/Source/gd32f50x_trigsel.c
    ${DRIVERS_DIR}/GD32F50x_standard_peripheral/Source/gd32f50x_trng.c
    ${DRIVERS_DIR}/GD32F50x_standard_peripheral/Source/gd32f50x_usart.c
    ${DRIVERS_DIR}/GD32F50x_standard_peripheral/Source/gd32f50x_wwdgt.c
    )

target_include_directories(GD32F50x_standard_peripheral PUBLIC
    ${DRIVERS_DIR}/GD32F50x_standard_peripheral/Include
    )

# CMSIS header only library is linked.
target_link_libraries(GD32F50x_standard_peripheral PUBLIC CMSIS)
