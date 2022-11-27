#include "MCPWM_Motor.h"

MCPWM_Motor::MCPWM_Motor(mcpwm_unit_t unit,
                mcpwm_timer_t timer,
                mcpwm_io_signals_t pwma_signal,
                mcpwm_io_signals_t pwmb_signal,
                gpio_num_t pina,
                gpio_num_t pinb)
{
    mcpwm_unit = unit;
    timer_num = timer;

    mcpwm_gpio_init(unit, pwma_signal, pina);
    mcpwm_gpio_init(unit, pwmb_signal, pinb);

    mcpwm_config_t conf = {
        .frequency = _frequency,
        .cmpr_a = 0,
        .cmpr_b = 0,
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER,
    };
    mcpwm_init(unit, timer_num, &conf);
}

void MCPWM_Motor::setFrequency(uint32_t freq)
{
    _frequency = freq;
}

void MCPWM_Motor::_setDuty(float duty, mcpwm_generator_t gen)
{
    mcpwm_set_duty(mcpwm_unit, timer_num, gen, duty);
    mcpwm_set_duty_type(mcpwm_unit, timer_num, gen, MCPWM_DUTY_MODE_0);
}

void MCPWM_Motor::setDuty(float duty)
{
    if(duty > 100) duty = 100;
    else if (duty < -100) duty = -100;
    if (duty >= 0)
    {
        mcpwm_set_signal_low(mcpwm_unit, timer_num, MCPWM_GEN_A);
        _setDuty(duty, MCPWM_GEN_B);
    }
    else
    {
        mcpwm_set_signal_low(mcpwm_unit, timer_num, MCPWM_GEN_B);
        _setDuty(-duty, MCPWM_GEN_A);
    }
}
