#include "rotary_encoder.h"

// A lock to avoid pcnt isr service being installed twice in multiple threads.
static _lock_t isr_service_install_lock;
#define LOCK_ACQUIRE() _lock_acquire(&isr_service_install_lock)
#define LOCK_RELEASE() _lock_release(&isr_service_install_lock)

namespace ROTARY_ENCODER
{
    // Static variable initializations
    bool RotaryEncoder::_interrupt_service_installed{false};

    esp_err_t RotaryEncoder::_init(pcnt_unit_t unit, gpio_num_t CHA, gpio_num_t CHB, bool reverse)
    {
        esp_err_t status{ESP_OK};

        _interrupt_args.pcnt_unit = unit;

        // Configure channel 0
        pcnt_config_t pcnt_conf = {
            .pulse_gpio_num = CHA,
            .ctrl_gpio_num = CHB,
            .lctrl_mode = PCNT_MODE_REVERSE,
            .hctrl_mode = PCNT_MODE_KEEP,
            .pos_mode = reverse ? PCNT_COUNT_INC : PCNT_COUNT_DEC,
            .neg_mode = reverse ? PCNT_COUNT_DEC : PCNT_COUNT_INC,
            .counter_h_lim = _interrupt_args.PCNT_HIGH_LIMIT,
            .counter_l_lim = _interrupt_args.PCNT_LOW_LIMIT,
            .unit = _interrupt_args.pcnt_unit,
            .channel = PCNT_CHANNEL_0,
        };

        status |= pcnt_unit_config(&pcnt_conf);

        // Configure channel 1
        pcnt_conf.pulse_gpio_num = CHB;
        pcnt_conf.ctrl_gpio_num = CHA;
        pcnt_conf.channel = PCNT_CHANNEL_1;
        pcnt_conf.pos_mode = reverse ? PCNT_COUNT_DEC : PCNT_COUNT_INC;
        pcnt_conf.neg_mode = reverse ? PCNT_COUNT_INC : PCNT_COUNT_DEC;
        status |= pcnt_unit_config(&pcnt_conf);

        // PCNT pause and reset value
        pcnt_counter_pause(_interrupt_args.pcnt_unit);
        pcnt_counter_clear(_interrupt_args.pcnt_unit);

        // register interrupt handler in a thread-safe way
        LOCK_ACQUIRE();
        if (!_interrupt_service_installed)
        {
            status = pcnt_isr_service_install(0);
            if (status == ESP_OK)
            {
                _interrupt_service_installed = true;
            }            
        }
        LOCK_RELEASE();

        pcnt_isr_handler_add(_interrupt_args.pcnt_unit, overflow_handler_isr, (void*) &_interrupt_args);

        pcnt_event_enable(_interrupt_args.pcnt_unit, PCNT_EVT_H_LIM);
        pcnt_event_enable(_interrupt_args.pcnt_unit, PCNT_EVT_L_LIM);

        return status;
    }

    RotaryEncoder::RotaryEncoder(pcnt_unit_t unit, gpio_num_t CHA, gpio_num_t CHB, int16_t HIGH_LIMIT, int16_t LOW_LIMIT, bool reverse)
    {
        //_interrupt_args.accumu_count = 500;

        _interrupt_args.PCNT_HIGH_LIMIT = HIGH_LIMIT;
        _interrupt_args.PCNT_LOW_LIMIT = LOW_LIMIT;
        ESP_ERROR_CHECK(_init(unit, CHA, CHB, reverse));
    }

    void IRAM_ATTR RotaryEncoder::overflow_handler_isr(void *args)
    {
        uint32_t status = 0;
        pcnt_unit_t pcnt_unit = (reinterpret_cast<struct interrupt_args *>(args))->pcnt_unit;
        int32_t & accumu_count = (reinterpret_cast<struct interrupt_args *>(args))->accumu_count;

        pcnt_get_event_status(pcnt_unit, &status);

        if (status & PCNT_EVT_H_LIM) {
            accumu_count += (reinterpret_cast<struct interrupt_args *>(args))->PCNT_HIGH_LIMIT;
        } else if (status & PCNT_EVT_L_LIM) {
            accumu_count += (reinterpret_cast<struct interrupt_args *>(args))->PCNT_LOW_LIMIT;
        }
    }

    esp_err_t RotaryEncoder::set_glitch_filter(uint32_t max_glitch_us)
    {
        esp_err_t status{ESP_OK};

        /* Configure and enable the input filter */
        status |= pcnt_set_filter_value(_interrupt_args.pcnt_unit, max_glitch_us * 80);

        if (max_glitch_us) {
            pcnt_filter_enable(_interrupt_args.pcnt_unit);
        } else {
            pcnt_filter_disable(_interrupt_args.pcnt_unit);
        }
        return status;
    }

    esp_err_t RotaryEncoder::start(void)
    {
        esp_err_t status{ESP_OK};
        status |= pcnt_counter_resume(_interrupt_args.pcnt_unit);
        return status;
    }

    esp_err_t RotaryEncoder::stop(void)
    {
        esp_err_t status{ESP_OK};
        status |= pcnt_counter_pause(_interrupt_args.pcnt_unit);
        return status;
    }

    uint32_t RotaryEncoder::counter_value(void)
    {
        int16_t val = 0;
        pcnt_get_counter_value(_interrupt_args.pcnt_unit, &val);
        val += _interrupt_args.accumu_count;
        return val;
    }
}
