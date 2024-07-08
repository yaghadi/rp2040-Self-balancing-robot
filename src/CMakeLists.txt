add_executable(${ProjectName}
    main.c
    robot_control.c
    robot_control.h
    mpu6050.c
    mpu6050.h
    pico_servo.c
    pico_servo.h 
    max7219.c 
    max7219.h 
    hc-05.c 
    hc-05.h
)

target_include_directories(${ProjectName} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(${ProjectName} 
    pico_stdlib 
    hardware_i2c
    hardware_spi
    hardware_uart
    hardware_pwm
    hardware_timer
    pico_multicore
    m
    FreeRTOS-Kernel-Heap4 
    )

# Generate the UF2 file
pico_add_extra_outputs(${ProjectName})

# enable usb output, disable uart output
pico_enable_stdio_usb(${ProjectName} 0)
pico_enable_stdio_uart(${ProjectName} 1)