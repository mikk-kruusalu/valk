#include "driver/mcpwm.h"

class MCPWM_Motor
{
private:
    mcpwm_unit_t mcpwm_unit;
    mcpwm_timer_t timer_num;
    uint32_t _frequency = 20'000;

    void _setDuty(float duty, mcpwm_generator_t gen);
public:
    MCPWM_Motor(mcpwm_unit_t unit,
                mcpwm_timer_t timer,
                mcpwm_io_signals_t pwma_signal,
                mcpwm_io_signals_t pwmb_signal,
                gpio_num_t pina,
                gpio_num_t pinb);
    void setFrequency(uint32_t freq);
    
    void setDuty(float duty);
};
