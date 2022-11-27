#include "CPPLEDC/cppledc.h"

class LEDC_Motor
{
private:
    CPPLEDC::CppLedcTimer timer;
    CPPLEDC::CppLedc pwma;
    CPPLEDC::CppLedc pwmb;

public:
    LEDC_Motor(gpio_num_t pin1, gpio_num_t pin2, ledc_timer_t timer_num);
    void setDuty(int16_t duty);
};

