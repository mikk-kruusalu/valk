#pragma once

// WS2812 LED
#define LED_PIN     GPIO_NUM_19

// battery monitoring
#define BAT_ADC_CHANNEL ADC1_CHANNEL_7

// line sensor pins
#define SENSOR_CTRL_PIN GPIO_NUM_25
#define SENSOR_MUXA     GPIO_NUM_32
#define SENSOR_MUXB     GPIO_NUM_33
#define SENSOR_PIN0     ADC1_CHANNEL_0            //GPIO_NUM_36
#define SENSOR_PIN1     ADC1_CHANNEL_1            //GPIO_NUM_37
#define SENSOR_PIN2     ADC1_CHANNEL_2            //GPIO_NUM_38
#define SENSOR_PIN3     ADC1_CHANNEL_3            //GPIO_NUM_39

// buttons
#define BUTTON_PIN  GPIO_NUM_0

// EDF
#define EDF_PWM_PIN 21

// Motor driver pins
#define MOTA_PIN1      GPIO_NUM_12
#define MOTA_PIN2      GPIO_NUM_2
#define MOTB_PIN1      GPIO_NUM_23
#define MOTB_PIN2      GPIO_NUM_18

// Encoder pins
#define ENCA_PINA       GPIO_NUM_13
#define ENCA_PINB       GPIO_NUM_15
#define ENCB_PINA       GPIO_NUM_4
#define ENCB_PINB       GPIO_NUM_5

// I2C
#define SDA_PIN     GPIO_NUM_26
#define SCL_PIN     GPIO_NUM_27

// IMU
#define IMU_INT1_PIN -1
#define IMU_INT2_PIN 14
