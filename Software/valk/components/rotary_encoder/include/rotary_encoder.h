#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_event.h"
#include "driver/pcnt.h"


namespace ROTARY_ENCODER
{
    class RotaryEncoder
    {
    private:
        esp_err_t _init(pcnt_unit_t pcnt_unit, gpio_num_t CHA, gpio_num_t CHB, bool reverse);
        static bool _interrupt_service_installed;

        struct interrupt_args {
            pcnt_unit_t pcnt_unit;
            int32_t accumu_count;
            int16_t PCNT_HIGH_LIMIT;
            int16_t PCNT_LOW_LIMIT;
        } _interrupt_args;

    public:
        RotaryEncoder(pcnt_unit_t unit, gpio_num_t CHA, gpio_num_t CHB, int16_t HIGH_LIMIT, int16_t LOW_LIMIT, bool reverse = false);
        esp_err_t set_glitch_filter(uint32_t max_glitch_us);

        esp_err_t start(void);
        esp_err_t stop(void);
        uint32_t counter_value(void);
        static void IRAM_ATTR overflow_handler_isr(void *arg);
    };
}
