#pragma once

#include "CPPGPIO/cppgpio.h"
#include "CPPANALOG/analogio.h"
#include "CPPLEDC/cppledc.h"

#define NVS_CALIB_NAMESPACE "calibration"

namespace LINE_SENSOR
{
    class LineSensor
    {
    private:
        ledc_timer_t ledc_timer_num = LEDC_TIMER_0;
        uint8_t _sensorCount = 12;
        uint16_t _maxValue = 0;
        static const uint8_t NUM_ANALOG_PINS = 4;
        uint8_t _calib_samples_count = 100;

        uint16_t _lastPosition = 0;

        struct CalibrationData {
            uint16_t *max = nullptr;
            uint16_t *min = nullptr;
        } calibration_data;

        CPPLEDC::CppLedcTimer ledc_timer;
        CPPLEDC::CppLedc ledc;

        CPPGPIO::GpioOutput mux_cha;
        CPPGPIO::GpioOutput mux_chb;

        CPPANALOG::CppAdc1 analog_pins[NUM_ANALOG_PINS];

        void control_init(gpio_num_t pin);
        void mux_init(gpio_num_t cha, gpio_num_t chb);
        void analogio_init(adc1_channel_t * pins);
    public:
        LineSensor(uint8_t num_sensors, adc1_channel_t * analogio, gpio_num_t mux_cha, gpio_num_t mux_chb, gpio_num_t ctrl_pin);

        void read_raw(uint16_t * sensor_values);
        void set_emitter(uint32_t duty);

        void calibrate(void (*task_every_iter)(void));
        void read_calibrated(uint16_t * sensor_values);
        uint16_t read_line(uint16_t * sensor_values, uint16_t * sensor_bin, bool * on_line);
        uint8_t read_calibration_nvs();
        void save_calibration_nvs();
    };
}
