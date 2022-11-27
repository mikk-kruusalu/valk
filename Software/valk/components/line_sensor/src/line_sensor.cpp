#include "line_sensor.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char* TAG = "Line sensor";

namespace LINE_SENSOR
{
    void LineSensor::control_init(gpio_num_t pin)
    {
        ledc_timer.InitializeDefaults();
        ledc_timer.SetTimerDutyResolution(LEDC_TIMER_8_BIT);

        ledc.SetChannelGpio(pin);
        ledc.SetChannelTimerSource(ledc_timer_num);
        ledc.SetChannelDutyCycle(0);
    }

    void LineSensor::mux_init(gpio_num_t cha, gpio_num_t chb)
    {
        mux_cha.init(cha);
        mux_chb.init(chb);
    }

    void LineSensor::analogio_init(adc1_channel_t * pins)
    {
        _maxValue = 4095;
        for(int i=0; i < NUM_ANALOG_PINS; i++)
        {
            CPPANALOG::CppAdc1 pin{pins[i]};
            pin.SetBitWidth(12);
            pin.SetAttenuation(ADC_ATTEN_DB_11);
            analog_pins[i] = pin;
        }
    }

    LineSensor::LineSensor(uint8_t num_sensors, adc1_channel_t * analogio, gpio_num_t mux_cha, gpio_num_t mux_chb, gpio_num_t ctrl_pin)
    {
        _sensorCount = num_sensors;
        control_init(ctrl_pin);
        mux_init(mux_cha, mux_chb);
        analogio_init(analogio);
    }

    void LineSensor::read_raw(uint16_t * sensor_values)
    {
        mux_cha.off();
        mux_chb.off();
        sensor_values[9] = analog_pins[0].GetRaw();
        sensor_values[6] = analog_pins[1].GetRaw();
        sensor_values[5] = analog_pins[2].GetRaw();
        sensor_values[0] = analog_pins[3].GetRaw();

        mux_cha.on();
        mux_chb.off();
        sensor_values[11] = analog_pins[0].GetRaw();
        sensor_values[8] = analog_pins[1].GetRaw();
        sensor_values[3] = analog_pins[2].GetRaw();
        sensor_values[1] = analog_pins[3].GetRaw();

        mux_cha.off();
        mux_chb.on();
        sensor_values[10] = analog_pins[0].GetRaw();
        sensor_values[7] = analog_pins[1].GetRaw();
        sensor_values[4] = analog_pins[2].GetRaw();
        sensor_values[2] = analog_pins[3].GetRaw();
    }

    void LineSensor::set_emitter(uint32_t duty)
    {
        ledc.SetChannelDutyCycle(duty);
    }

    void LineSensor::calibrate(void (*task_every_iter)(void))
    {
        uint16_t sensorValues[_sensorCount];

        calibration_data.max = (uint16_t *)realloc(calibration_data.max,
                                                sizeof(uint16_t) * _sensorCount);

        calibration_data.min = (uint16_t *)realloc(calibration_data.min,
                                                sizeof(uint16_t) * _sensorCount);

        // Initialize the max and min calibrated values to values that
        // will cause the first reading to update them.
        for (uint8_t i = 0; i < _sensorCount; i++)
        {
            calibration_data.max[i] = 0;
            calibration_data.min[i] = _maxValue;
        }
        ESP_LOGI(TAG, "Calibrated values:");
        for(uint8_t i = 0; i < _sensorCount; i++)
        {
            ESP_LOGI(TAG, "%d min: %d \t max: %d", i, calibration_data.min[i], calibration_data.max[i]);
        }

        for(uint8_t j=0; j<_calib_samples_count; j++)
        {
            read_raw(sensorValues);

            for (uint8_t i = 0; i < _sensorCount; i++)
            {
                // set the max we found THIS time
                if ((j == 0) || (sensorValues[i] > calibration_data.max[i]))
                {
                    calibration_data.max[i] = sensorValues[i];
                }

                // set the min we found THIS time
                if ((j == 0) || (sensorValues[i] < calibration_data.min[i]))
                {
                    calibration_data.min[i] = sensorValues[i];
                }
            }
            task_every_iter();
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        ESP_LOGI(TAG, "Calibrated values:");
        for(uint8_t i = 0; i < _sensorCount; i++)
        {
            ESP_LOGI(TAG, "%d min: %d \t max: %d", i, calibration_data.min[i], calibration_data.max[i]);
        }
    }

    void LineSensor::read_calibrated(uint16_t * sensor_values)
    {
        read_raw(sensor_values);

        for (uint8_t i=0; i<_sensorCount; i++)
        {
            uint16_t denominator = calibration_data.max[i] - calibration_data.min[i];
            int16_t value = 0;

            if (denominator != 0)
            {
                value = (((int32_t)sensor_values[i]) - calibration_data.min[i]) * 1000 / denominator;
            }

            if (value < 0) { value = 0; }
            else if (value > 1000) { value = 1000; }

            sensor_values[i] = value;
        }
    }

    uint16_t LineSensor::read_line(uint16_t * sensor_values, uint16_t * sensor_bin, bool * on_line)
    {
        bool onLine = false;
        uint32_t avg = 0; // this is for the weighted total
        uint16_t sum = 0; // this is for the denominator, which is <= 64000

        read_calibrated(sensor_values);

        for (uint8_t i = 0; i < _sensorCount; i++)
        {
            uint16_t value = sensor_values[i];

            // keep track of whether we see the line at all
            if(sensor_values[i] > 300) 
            {
                onLine = true;
                (*sensor_bin) |= 1 << i;
            }
            else (*sensor_bin) &= ~(1 << i);

            // only average in values that are above a noise threshold
            if (value > 50)
            {
                avg += (uint32_t)value * (i * 1000);
                sum += value;
            }
        }
        
        (*on_line) = onLine;

        if (!onLine)
        {
            // If it last read to the left of center, return 0.
            if (_lastPosition < (_sensorCount - 1) * 1000 / 2)
            {
                return 0;
            }
            // If it last read to the right of center, return the max.
            else
            {
                return (_sensorCount - 1) * 1000;
            }
        }

        _lastPosition = avg / sum;
        return _lastPosition;
    }

    void LineSensor::save_calibration_nvs()
    {
        nvs_handle_t my_handle;
        esp_err_t err;

        // Open
        err = nvs_open(NVS_CALIB_NAMESPACE, NVS_READWRITE, &my_handle);
        if (err != ESP_OK) ESP_LOGE(TAG, "NVS open failed!"); //return err;

        size_t required_size = 2 * _sensorCount * sizeof(uint16_t);

        uint16_t data[2*_sensorCount];
        for (uint8_t i=0; i < _sensorCount; i++)
        {
            data[i] = calibration_data.min[i];
            data[i + _sensorCount] = calibration_data.max[i];
        }
        
        ESP_LOGI(TAG, "Write calibration values to nvs %d bytes", required_size);
        for (int i = 0; i < 2*_sensorCount; i++) {
            ESP_LOGI(TAG, "%d: %d", i + 1, data[i]);
        }

        err = nvs_set_blob(my_handle, NVS_CALIB_NAMESPACE, data, required_size);

        if (err != ESP_OK) ESP_LOGE(TAG, "NVS write failed!");

        // Commit
        err = nvs_commit(my_handle);
        if (err != ESP_OK) ESP_LOGE(TAG, "NVS commit failed!");

        // Close
        nvs_close(my_handle);
    }

    uint8_t LineSensor::read_calibration_nvs()
    {
        nvs_handle_t my_handle;

        // Open
        esp_err_t err = nvs_open(NVS_CALIB_NAMESPACE, NVS_READWRITE, &my_handle);
        if (err != ESP_OK) ESP_LOGE(TAG, "NVS open failed!"); //return err;

        // Read run time blob
        size_t required_size = 0;  // value will default to 0, if not set yet in NVS
        // obtain required memory space to store blob being read from NVS
        err = nvs_get_blob(my_handle, NVS_CALIB_NAMESPACE, NULL, &required_size);
        if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) ESP_LOGE(TAG, "NVS get_blob failed!"); //return err;

        if (required_size == 0) {
            ESP_LOGI(TAG, "No previous calibration values found, needs calibration!");
            return 1;
        } 
        else {
            uint16_t* calib_data = (uint16_t *)malloc(required_size);
            err = nvs_get_blob(my_handle, NVS_CALIB_NAMESPACE, calib_data, &required_size);
            if (err != ESP_OK) {
                free(calib_data);
                ESP_LOGI(TAG, "NVS get blob failed!");
                return 1;
            }

            ESP_LOGI(TAG, "Read calibration values from NVS:");
            calibration_data.max = (uint16_t *)realloc(calibration_data.max,
                                                sizeof(uint16_t) * _sensorCount);
            calibration_data.min = (uint16_t *)realloc(calibration_data.min,
                                                sizeof(uint16_t) * _sensorCount);
            for(uint8_t i = 0; i < _sensorCount; i++)
            {
                calibration_data.min[i] = calib_data[i];
                calibration_data.max[i] = calib_data[i + _sensorCount];
                ESP_LOGI(TAG, "%d min: %d \t max: %d", i, calibration_data.min[i], calibration_data.max[i]);
            }

            free(calib_data);
        }
        // Close
        nvs_close(my_handle);
        return 0;
    }
}
