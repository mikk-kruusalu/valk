# pragma once

//in ms
#define MC_TASK_PERIOD  10
#define LF_TASK_PERIOD  10

// motor PID
#define MOTOR_KP   0.1
#define MOTOR_KI   0.00022 * MC_TASK_PERIOD
#define MOTOR_KD   0 / MC_TASK_PERIOD
#define MOTOR_KF   0.011

// rataste vahe 155mm, tagumikust sensori otsani 145mm
#define LINE_KP     0.3
#define LINE_KI     0 * LF_TASK_PERIOD
#define LINE_KD     1.2 / LF_TASK_PERIOD
#define LINE_KF     0
#define AVG_SPEED   1400    // mm/s
#define MAX_SPEED   3000

#define MIN_BAT_MV   6200
#define EMITTER_LEVEL   80


/*ONLY FOR BIG WHEELS*/
// #define LINE_KP     0.6
// #define LINE_KI     0
// #define LINE_KD     5
// #define LINE_KF     0
// #define AVG_SPEED   3000
// #define MAX_SPEED   3000
/*BIG WHEELS END*/



// speeds
// acceleration is in mm/s^2
// xspeed is measured in mm/s, w speed in deg/s

// All measurements are in mm
#define WHEEL_RADIUS        13.5
#define WHEEL_TO_WHEEL      150
#define WHEELS_TO_SENSORS   150
#define PI                  3.14

// Encoders
#define TICKS_PER_REV       52
#define MM_PER_TICK         ((2*PI*WHEEL_RADIUS)/TICKS_PER_REV)
#define DEG_PER_TICK        ((360 * MM_PER_TICK)/(PI * WHEEL_TO_WHEEL))
#define RAD_PER_TICK        ((2*MM_PER_TICK)/WHEEL_TO_WHEEL)

#define TICKS_TO_RAD(x)     ((x)*(2*PI/TICKS_PER_REV))
#define TICKS_TO_MM(x)      (((2*PI*WHEEL_RADIUS)/TICKS_PER_REV)*(x))
#define MM_TO_TICKS(x)      ((TICKS_PER_REV/(2*PI*WHEEL_RADIUS))*(x))






// // I2C
// #define IMU_I2C_CLK_SPEED_HZ    400000
// #define IMU_I2C_PORT            I2C_NUM_0
// #define I2C_MASTER_TIMEOUT  1000    //ms


// // IMU
// #define ACCEL_FS        LSM6DS3_2g
// #define GYRO_FS         LSM6DS3_500dps
// #define ACCEL_ODR       LSM6DS3_XL_ODR_1k66Hz
// #define GYRO_ODR        LSM6DS3_GY_ODR_1k66Hz
// #define IMU_FIFO_ODR    LSM6DS3_FIFO_1k66Hz
// #define IMU_FIFO_PATTERN_LEN    12
// #define IMU_FIFO_NUM_PATTERNS   100
// #define GYRO_HP_BANDWIDTH   LSM6DS3_HP_CUT_OFF_2Hz07
// #define ACC_LP_BANDWIDTH    LSM6DS3_XL_LP_ODR_DIV_50

// // ADC
// #define ADC_ATTEN       ADC_ATTEN_DB_11
