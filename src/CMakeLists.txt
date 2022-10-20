project(uart_iqrf)

set(_SRC_FILES iqrf_gpio.c uart_iqrf.c)

add_library(${PROJECT_NAME} STATIC ${_SRC_FILES})
