set(PROJECT_NAME uart_example_dpa)

set(UART_EXAMPLE_DPA_SRC_FILES
        uart_example_dpa.c
        )

include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(${PROJECT_NAME} ${UART_EXAMPLE_DPA_SRC_FILES})
target_link_libraries(${PROJECT_NAME}
        sysfs_gpio
        uart_iqrf)

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/sbin)
