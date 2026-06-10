idf_component_register(SRCS "robot_firmware.cpp" "motor_control.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES freertos driver)