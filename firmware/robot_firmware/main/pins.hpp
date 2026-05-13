#ifndef PINS_HPP
#define PINS_HPP


//MDD3A #1 - LEFT side motors
//Motor 1 (front left)
#define MOTOR_FL_A 4
#define MOTOR_FL_B 5

//Motor 2 (rear left)
#define MOTOR_RL_A 27
#define MOTOR_RL_B 32

//MDD3A #2 - RIGHT side motors
//Motor 3 (front right)
#define MOTOR_FR_A 25
#define MOTOR_FR_B 26

//Motor 4 (rear right)
#define MOTOR_RR_A 18
#define MOTOR_RR_B 19

// Encoders - signal pin per motor
#define ENC_FL 34
#define ENC_FR 35
#define ENC_RL 36
#define ENC_RR 39

//I2C — MPU-6050
#define IMU_SDA 21
#define IMU_SCL 22

//UART2 — GPS
#define GPS_RX 16
#define GPS_TX 17

#define KILL_SWITCH 13
#endif