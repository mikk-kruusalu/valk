#include "LEDC_Motor.h"

LEDC_Motor::LEDC_Motor(gpio_num_t pin1, gpio_num_t pin2, ledc_timer_t timer_num)
{
    timer.SetTimerSource(timer_num);
    timer.SetTimerDutyResolution(LEDC_TIMER_8_BIT);

    pwma.SetChannelGpio(pin1);
    pwmb.SetChannelGpio(pin2);

    pwma.SetChannelTimerSource(timer_num);
    pwmb.SetChannelTimerSource(timer_num);
}

void LEDC_Motor::setDuty(int16_t duty)
{
    if (duty >= 0)
    {
        pwma.SetChannelDutyCycle(duty);
        pwmb.SetChannelDutyCycle(0);
    }
    else
    {
        pwma.SetChannelDutyCycle(0);
        pwmb.SetChannelDutyCycle(duty);
    }
}
