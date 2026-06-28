idf_component_register(SRCS "robot_firmware.cpp" "motor_control.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES freertos driver esp_hw_support esp_wifi esp_netif nvs_flash esp_event micro_ros_espidf_component esp_timer)
