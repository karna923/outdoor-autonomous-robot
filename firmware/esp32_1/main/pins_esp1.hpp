#ifndef PINS_ESP1_HPP
#define PINS_ESP1_HPP


//MDD3A #1 - FRONT side motors
//Motor 1 (front left)
#define MOTOR_FL_A 4
#define MOTOR_FL_B 5

//Motor 2 (front right)
#define MOTOR_FR_A 13
#define MOTOR_FR_B 18

//MDD3A #2 - MID side motors
//Motor 3 (mid left)
#define MOTOR_ML_A 19
#define MOTOR_ML_B 23

//Motor 4 (mid right)
#define MOTOR_MR_A 25
#define MOTOR_MR_B 26

// Encoders - signal pin per motor
#define ENC_FL 34
#define ENC_FR 35
#define ENC_ML 36
#define ENC_MR 39

//I2C — MPU-6050
#define IMU_SDA 21
#define IMU_SCL 22

//UART2 — GPS
#define GPS_RX 16
#define GPS_TX 17

#define KILL_SWITCH 33
#endif