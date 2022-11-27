#include <iostream>
#include <bitset>
#include <stdio.h>

#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "led_strip.h"
#include "iot_button.h"

static const char* TAG = "Main";

// FreeRTOS components
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// peripheral wrappers
#include "CPPGPIO/cppgpio.h"
#include "CPPANALOG/analogio.h"

// configuration
#include "pins.h"
#include "configuration.h"

// custom components
#include "rotary_encoder.h"
#include "MiniPID.h"
#include "line_sensor.h"
#include "MCPWM_Motor.h"
#include "VL53L0X.h"
//#include "LSM6DS3.h"

// Main modes of the robot
enum robot_modes {
    IDLE,
    CALIBRATION,
    RUN
} robot_mode;

// speed is measured in mm/s
struct motion_control {
    int16_t target_lspeed = 0;
    int16_t target_rspeed = 0;

    int16_t cur_lspeed = 0;
    int16_t cur_rspeed = 0;

    uint16_t base_speed = AVG_SPEED;
    uint16_t max_speed = MAX_SPEED;
} mc;

// motor objects for setting speed
MCPWM_Motor right_motor{MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM0B, MOTA_PIN1, MOTA_PIN2};
MCPWM_Motor left_motor{MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM1A, MCPWM1B, MOTB_PIN2, MOTB_PIN1};

// PID control objects for adjusting motos wspeed
MiniPID left_PID{MOTOR_KP, MOTOR_KI, MOTOR_KD, MOTOR_KF};
MiniPID right_PID{MOTOR_KP, MOTOR_KI, MOTOR_KD, MOTOR_KF};

// rotary encoder objects for determining motor position
ROTARY_ENCODER::RotaryEncoder right_encoder{PCNT_UNIT_0, ENCA_PINA, ENCA_PINB, 100, -100, true};
ROTARY_ENCODER::RotaryEncoder left_encoder{PCNT_UNIT_1, ENCB_PINA, ENCB_PINB, 100, -100};

// button pin
static button_handle_t button = 0;
//CPPGPIO::GpioInput button{BUTTON_PIN};

// battery adc measurement, voltage measurement should be multiplyed by 2.61 beacause of voltage divider
CPPANALOG::CppAdc1 battery_adc{BAT_ADC_CHANNEL};
float bat_v = 0;

VL53L0X tof{I2C_NUM_0};
uint16_t tof_dist_mm = 1200;

// LSM6DS3 imu;
// float acc_mg[3];

// initialize line sensor
adc1_channel_t sensor_pins[4] = {SENSOR_PIN0, SENSOR_PIN1, SENSOR_PIN2, SENSOR_PIN3};
LINE_SENSOR::LineSensor line_sensor{12, sensor_pins, SENSOR_MUXA, SENSOR_MUXB, SENSOR_CTRL_PIN};
MiniPID line_PID{LINE_KP, LINE_KI, LINE_KD, LINE_KF};

static led_strip_t *rgb;

//Tasks
TaskHandle_t mc_task_h;
TaskHandle_t lf_task_h;
TaskHandle_t line_pid_task_h;
TaskHandle_t read_tof_h;

float update_battery_voltage()
{
    bat_v = battery_adc.GetVoltage() * 2.61;
    return bat_v;
}

void set_led(uint32_t red, uint32_t green, uint32_t blue)
{
    rgb->set_pixel(rgb, 0, red, green, blue);
    rgb->refresh(rgb, 100);
}

void clamp(int16_t & x, int32_t max, int32_t min)
{
    if (x > max) x = max;
    else if (x < min) x = min;
}

void check_battery()
{
    update_battery_voltage();
    if(bat_v < MIN_BAT_MV)
    {
        ESP_LOGW(TAG, "Battery voltage too low: %.2f \t Quitting...", bat_v);
        left_motor.setDuty(0);
        right_motor.setDuty(0);
        while(1){
            set_led(255, 0, 0);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            set_led(0, 0, 0);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
    //ESP_LOGI(TAG, "Battery healthy: %.2f", bat_v);
}


// should be run every MC_TASK_PERIOD
void motion_control_task(void *parameter)
{
    int32_t right_count = 0;
    int32_t left_count = 0;
    int32_t prev_right_count = 0;
    int32_t prev_left_count = 0;

    float left_duty = 0;
    float right_duty = 0;

    uint32_t collision_timer = 0;
    uint16_t collision_thresh = 300;

    while(1) {
        check_battery();

        clamp(mc.target_lspeed, MAX_SPEED, -MAX_SPEED);
        left_PID.setSetpoint(mc.target_lspeed);
        clamp(mc.target_rspeed, MAX_SPEED, -MAX_SPEED);
        right_PID.setSetpoint(mc.target_rspeed);

        /*ONLY FOR BIG WHEELS*/
        // mc.target_lspeed /= 30;
        // mc.target_rspeed /= 30;
        // right_motor.setDuty(mc.target_rspeed);
        // left_motor.setDuty(mc.target_lspeed);
        /*BIG WHEELS END*/
        

        // get encoder count
        right_count = right_encoder.counter_value();
        left_count = left_encoder.counter_value();

        // calculate current speeds of the wheels
        mc.cur_lspeed = 1000 * MM_PER_TICK * (left_count - prev_left_count) / MC_TASK_PERIOD;
        mc.cur_rspeed = 1000 * MM_PER_TICK * (right_count - prev_right_count) / MC_TASK_PERIOD;

        // set duty cycle of motors
        right_duty = right_PID.getOutput(mc.cur_rspeed);
        left_duty = left_PID.getOutput(mc.cur_lspeed);
        right_motor.setDuty(right_duty);
        left_motor.setDuty(left_duty);
        
        // check if motors are stuck
        if((left_duty > 90 || left_duty < -90) || (right_duty > 90 || right_duty < -90))
        {
            if(collision_timer == 0) collision_timer = xTaskGetTickCount();
        } else {
            collision_timer = 0;
        }
        if(collision_timer !=0 && (xTaskGetTickCount() - collision_timer) >= collision_thresh)
        {
            ESP_LOGW(TAG, "I got stuck :(");
            left_motor.setDuty(0);
            right_motor.setDuty(0);
            collision_timer = 0;
            while(1){
                set_led(100, 0, 100);
                vTaskDelay(300 / portTICK_PERIOD_MS);
                set_led(0, 0, 0);
                vTaskDelay(300 / portTICK_PERIOD_MS);
            }
        }

        // save encoder count for next run
        prev_left_count = left_count;
        prev_right_count = right_count;

        vTaskDelay(MC_TASK_PERIOD / portTICK_RATE_MS);
    }
}

void print_sensor_bin(uint16_t & binary_value)
{
    std::bitset<16> bs(binary_value);
    std::cout << "0b" << bs << std::endl;
}

void read_tof_dist(void *args)
{
    tof.setTimingBudget(20000);
    while(1)
    {
        if(!tof.read(&tof_dist_mm))
        {
            tof_dist_mm = 1200;
            //ESP_LOGE(TAG, "TOF measurement failed!");
        }
        //imu.readAcceleration(acc_mg[0], acc_mg[1], acc_mg[2]);
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
}

// variables for line following
struct line_follow{
    uint16_t position = 0;
    uint16_t last_position = 0;
    double PID_out = 0;

    uint16_t sensor_values[12];
    uint16_t sensor_bin = 0;    // bits 0 to 11 represent sensors, 12 to 15 are always zero
    uint16_t last_sensor_bin = 0;
    bool on_line = false;
} lf;

void update_line_pid(void)
{
    lf.PID_out = line_PID.getOutput(lf.position);
    mc.target_lspeed = mc.base_speed - lf.PID_out;
    mc.target_rspeed = mc.base_speed + lf.PID_out;
    
    clamp(mc.target_lspeed, mc.max_speed, 0);
    clamp(mc.target_rspeed, mc.max_speed, 0);
}

void line_follow(void *args)
{
    enum Obstacles {
        NONE,
        CUT,
        WALL,
        SWING,
        HILL,
        LOOP,
        Y_LEFT,
        Y_RIGHT,
        SPEED_LIMIT
    } obstacle;
    obstacle = NONE;

    uint32_t cut_timer = 0;
    uint32_t y_left_timer = 0;
    uint32_t y_right_timer = 0;
    uint32_t speed_limit_timer = 0;
    uint32_t hill_timer = 0;

    // variables for tuning
    const uint8_t cut_thresh = 20;
    const uint8_t y_thresh = 20;
    const uint8_t speed_limit_thresh = 30;
    const uint16_t wall_thresh = 210;

    // const uint16_t cut[5] =         {0b0000000001100000, 0b0000000001000000, 0b0000000011000000, 0b0000000000100000, 0b0000000000110000};
    // const uint16_t y_left[6] =      {0b0000000001100110, 0b0000000001100010, 0b0000000011001100, 0b0000000001001100, 0b0000000000110011, 0b0000000000110001};
    // const uint16_t y_right[6] =     {0b0000011001100000, 0b0000010001100000, 0b0000001100110000, 0b0000001100100000, 0b0000110011000000, 0b0000100011000000};
    // const uint16_t speed_limit[5] = {0b0000011001100110, 0b0000011001100100, 0b0000001001100110, 0b0000001101100110, 0b0000011001101100};

    line_PID.setSetpoint(5500);

    //uint32_t timer = 0;

    while(1)
    {
        //ESP_LOGI(TAG, "%d, %d", configMINIMAL_STACK_SIZE, uxTaskGetStackHighWaterMark(NULL));
        // ESP_LOGI(TAG, "%d", (timer - xTaskGetTickCount()));
        // timer = xTaskGetTickCount();
        switch(obstacle)
        {
            case NONE:
                // wall in front
                //ESP_LOGI(TAG, "Dist: %d mm \t Z acc: %f mg", tof_dist_mm, acc_mg[2]);
                if(tof_dist_mm <= wall_thresh && obstacle != HILL)
                {
                    mc.target_lspeed = 0;
                    mc.target_rspeed = 0;
                    vTaskDelay(800 / portTICK_PERIOD_MS);

                    if(tof_dist_mm <= wall_thresh)
                    {
                        ESP_LOGI(TAG, "Obstacle WALL");
                        obstacle = WALL;
                        break;
                    }
                    // obstacle = HILL;
                    // ESP_LOGI(TAG, "Obstacle HILL");
                    // mc.target_lspeed = -800;
                    // mc.target_rspeed = -800;
                    // vTaskDelay(300 / portTICK_PERIOD_MS);
                    // hill_timer = xTaskGetTickCount();
                }
                lf.position = line_sensor.read_line(lf.sensor_values, &lf.sensor_bin, &lf.on_line);
                //ESP_LOGI(TAG, "%d", lf.position);
                // line_sensor.read_raw(sensor_values);
                // for (uint8_t i = 0; i < 12; i++)
                // {
                //     std::cout << sensor_values[i] << "\t";
                // }
                // std::cout << std::endl;

                if(lf.on_line)
                {
                    update_line_pid();

                    uint8_t sets_of_ones = 0;
                    uint8_t num_of_ones = 0;
                    uint8_t ones_midpoint = 0;
                    bool last_bit = false;
                    for (uint8_t i = 0; i < 12; i++) {
                        if ((lf.sensor_bin >> i) & 1) {
                            num_of_ones++;
                            if (!last_bit) {
                                sets_of_ones++;
                            }
                            ones_midpoint += i;
                            last_bit = true;
                        }
                        else {
                            last_bit = false;
                        }
                    }
                    ones_midpoint /= num_of_ones;

                    // start cut timer, we have to be on the line for some time before cut
                    if(sets_of_ones == 1 && ones_midpoint >= 4 && ones_midpoint <= 6) {                                                                         //lf.sensor_bin == cut[0] || lf.sensor_bin == cut[1] || lf.sensor_bin == cut[2] || lf.sensor_bin == cut[3] || lf.sensor_bin == cut[4]) {
                        if(cut_timer == 0) cut_timer = xTaskGetTickCount();
                    } else {  // if the robot is off the centre then reset cut timer
                        cut_timer = 0;
                    }

                    //print_sensor_bin(lf.sensor_bin);
                    // detect Y intersection left
                    if(sets_of_ones == 2 && ones_midpoint <= 5)                                                                                     //lf.sensor_bin == y_left[0] || lf.sensor_bin == y_left[1] || lf.sensor_bin == y_left[2] || lf.sensor_bin == y_left[3] || lf.sensor_bin == y_left[4] || lf.sensor_bin == y_left[5])
                    {
                        if(y_left_timer == 0) y_left_timer = xTaskGetTickCount();
                        if(y_left_timer == 1) break;
                        if ((xTaskGetTickCount() - y_left_timer) >= y_thresh)
                        {
                            obstacle = Y_LEFT;
                            ESP_LOGI(TAG, "Obstacle Y left");
                            y_left_timer = xTaskGetTickCount();
                            line_PID.setSetpoint(9000);
                            mc.base_speed = 800;
                            break;
                        }
                    } else {
                        if(y_left_timer != 1) y_left_timer = 0;
                    }

                    // detect Y intersection right
                    if(sets_of_ones == 2 && ones_midpoint >= 6)                                                                 //lf.sensor_bin == y_right[0] || lf.sensor_bin == y_right[1] || lf.sensor_bin == y_right[2] || lf.sensor_bin == y_right[3] || lf.sensor_bin == y_right[4] || lf.sensor_bin == y_right[5])
                    {
                        if(y_right_timer == 0) y_right_timer = xTaskGetTickCount();
                        if ((xTaskGetTickCount() - y_right_timer) >= y_thresh)
                        {
                            obstacle = Y_RIGHT;
                            ESP_LOGI(TAG, "Obstacle Y right");
                            y_right_timer = xTaskGetTickCount();
                            line_PID.setSetpoint(2000);
                            mc.base_speed = 800;
                            break;
                        }
                    } else {
                        y_right_timer = 0;
                    }
                    
                    // // detect speed limit section
                    // if(sets_of_ones == 3)                                                             //lf.sensor_bin == speed_limit[0] || lf.sensor_bin == speed_limit[1] || lf.sensor_bin == speed_limit[2] || lf.sensor_bin == speed_limit[3] || lf.sensor_bin == speed_limit[4])
                    // {
                    //     if(speed_limit_timer == 0) speed_limit_timer = xTaskGetTickCount();
                    //     else if ((xTaskGetTickCount() - speed_limit_timer) >= speed_limit_thresh)
                    //     {
                    //         obstacle = SPEED_LIMIT;
                    //         ESP_LOGI(TAG, "Obstacle speed limit");
                    //         speed_limit_timer = xTaskGetTickCount();
                    //         mc.base_speed = 800;
                    //         break;
                    //     }
                    // } else {
                    //     speed_limit_timer = 0;
                    // }
                }
                else
                {
                    // if the line was in the middle for some time but now gone turn on CUT mode
                    if(cut_timer != 0 && (xTaskGetTickCount() - cut_timer) >= cut_thresh)
                    {
                        ESP_LOGI(TAG, "Obstacle CUT, timer %d", cut_timer);
                        cut_timer = 0;
                        obstacle = CUT;
                        print_sensor_bin(lf.last_sensor_bin);
                        mc.base_speed = 800;
                        line_PID.setD(0);
                        break;
                    }

                    // if we are off the line and there are no obstacles
                    //update_line_pid();
                    mc.target_lspeed = -80;
                    mc.target_rspeed = -80;
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    left_PID.reset();
                    right_PID.reset();

                    while(!lf.on_line)
                    {
                        lf.position = line_sensor.read_line(lf.sensor_values, &lf.sensor_bin, &lf.on_line);
                        if (lf.position <= 10)
                        {
                            mc.target_lspeed = -800;
                            mc.target_rspeed = 800;
                        } else {
                            mc.target_lspeed = 800;
                            mc.target_rspeed = -800;
                        }
                    }
                }
                break;

            case CUT:
                lf.position = line_sensor.read_line(lf.sensor_values, &lf.sensor_bin, &lf.on_line);
                
                if(lf.on_line)
                {
                    update_line_pid();

                    if(cut_timer == 0) cut_timer = xTaskGetTickCount();
                    if((xTaskGetTickCount() - cut_timer) >= 3*cut_thresh)
                    {
                        obstacle = NONE;
                        cut_timer = 0;
                        ESP_LOGI(TAG, "Back on the line!");
                        mc.base_speed = AVG_SPEED;
                        line_PID.setD(LINE_KD);
                    }
                }
                else
                {
                    mc.target_lspeed = mc.base_speed;
                    mc.target_rspeed = mc.base_speed;
                    cut_timer = 0;
                }
                break;
            
            case WALL:
                // turn right
                mc.target_lspeed = 0;
                mc.target_rspeed = -800;
                vTaskDelay(420 / portTICK_PERIOD_MS);
                // move forward
                mc.target_lspeed = 800;
                mc.target_rspeed = 800;
                vTaskDelay(480 / portTICK_PERIOD_MS);
                // turn left
                mc.target_lspeed = 0;
                mc.target_rspeed = 800;
                vTaskDelay(340 / portTICK_PERIOD_MS);
                // move forward
                mc.target_lspeed = 800;
                mc.target_rspeed = 800;
                vTaskDelay(440 / portTICK_PERIOD_MS);
                // turn left
                mc.target_lspeed = 0;
                mc.target_rspeed = 800;
                vTaskDelay(250 / portTICK_PERIOD_MS);
                mc.target_lspeed = 0;
                mc.target_rspeed = 0;
                vTaskDelay(300 / portTICK_PERIOD_MS);
                lf.on_line = false;
                while(!lf.on_line)
                {
                    lf.position = line_sensor.read_line(lf.sensor_values, &lf.sensor_bin, &lf.on_line);
                    // move forward
                    mc.target_lspeed = 800;
                    mc.target_rspeed = 800;
                }
                lf.on_line = false;
                vTaskDelay(200 / portTICK_PERIOD_MS);
                while(!lf.on_line)
                {
                    lf.position = line_sensor.read_line(lf.sensor_values, &lf.sensor_bin, &lf.on_line);
                    // move forward
                    mc.target_lspeed = 800;
                    mc.target_rspeed = -800;
                }
                mc.target_lspeed = 0;
                mc.target_rspeed = 0;
                vTaskDelay(400 / portTICK_PERIOD_MS);
                obstacle = NONE;
                ESP_LOGI(TAG, "Continue on the line!");
                break;
            case Y_LEFT:
                if ((xTaskGetTickCount() - y_left_timer) >= 500)
                {
                    obstacle = NONE;
                    ESP_LOGI(TAG, "Continue normally");
                    line_PID.setSetpoint(5500);
                    y_left_timer = 1;
                    mc.base_speed = AVG_SPEED;
                    break;
                }
                lf.position = line_sensor.read_line(lf.sensor_values, &lf.sensor_bin, &lf.on_line);

                update_line_pid();
                break;
            
            case Y_RIGHT:
                if ((xTaskGetTickCount() - y_right_timer) >= 500)
                {
                    obstacle = NONE;
                    ESP_LOGI(TAG, "Continue normally");
                    line_PID.setSetpoint(5500);
                    y_right_timer = 0;
                    mc.base_speed = AVG_SPEED;
                    break;
                }
                lf.position = line_sensor.read_line(lf.sensor_values, &lf.sensor_bin, &lf.on_line);

                update_line_pid();
                break;
            
            case SPEED_LIMIT:
                lf.position = line_sensor.read_line(lf.sensor_values, &lf.sensor_bin, &lf.on_line);

                if (speed_limit_timer != 0 && (xTaskGetTickCount() - speed_limit_timer) > 1000)
                {
                    mc.target_lspeed = 0;
                    mc.target_rspeed = 0;
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    speed_limit_timer = xTaskGetTickCount();
                }
                update_line_pid();
                break;
            case HILL:
                if(hill_timer !=0 && (xTaskGetTickCount() - hill_timer) >= 1500)
                {
                    obstacle = NONE;
                    ESP_LOGI(TAG, "Continue normally");
                    hill_timer = 0;
                }

                lf.position = line_sensor.read_line(lf.sensor_values, &lf.sensor_bin, &lf.on_line);
                update_line_pid();
                break;
            default:
                break;
        }
        
        lf.last_position = lf.position;
        lf.last_sensor_bin = lf.sensor_bin;
        vTaskDelay(LF_TASK_PERIOD / portTICK_PERIOD_MS);
    }
}

static void button_single_click_cb(void *handle, void *args)
{
    if (robot_mode == IDLE)
    {
        ESP_LOGI(TAG, "Changed robot mode to RUN");
        robot_mode = RUN;
        set_led(80, 0, 80);

        line_sensor.set_emitter(EMITTER_LEVEL);
        //imu.begin();

        vTaskDelay(500 / portTICK_PERIOD_MS);
        vTaskResume(lf_task_h);
        vTaskResume(mc_task_h);
        vTaskResume(read_tof_h);
    }
    else if (robot_mode == RUN)
    {
        robot_mode = IDLE;
        vTaskSuspend(mc_task_h);
        vTaskSuspend(lf_task_h);
        vTaskSuspend(read_tof_h);
        ESP_LOGI(TAG, "Robot mode changed to IDLE");

        line_sensor.set_emitter(0);
        //imu.end();
        set_led(0, 0, 0);
        mc.target_lspeed = 0;
        mc.target_rspeed = 0;
        left_motor.setDuty(0);
        right_motor.setDuty(0);
    }
}

void calibration_fade(void)
{
    static uint8_t max_value = 100;
    static uint8_t i = 0;
    set_led(0, i, max_value-i);
    i += max_value/100;
    if(i >= max_value) i=0;
}

static void button_long_press_start_cb(void *handle, void *args)
{
    robot_mode = CALIBRATION;

    ESP_LOGI(TAG, "Start calibration");
    line_sensor.set_emitter(EMITTER_LEVEL);
    line_sensor.calibrate(calibration_fade);
    line_sensor.set_emitter(0);
    ESP_LOGI(TAG, "Calibration done!");
    
    ESP_LOGI(TAG, "Saving calibration data to NVS");
    line_sensor.save_calibration_nvs();

    vTaskDelay(500 / portTICK_PERIOD_MS);
    set_led(0, 0, 0);
    robot_mode = IDLE;
}

extern "C" void app_main(void)
{
    robot_mode = IDLE;
    // initialize rgb led
    rgb = led_strip_init(0, LED_PIN, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    rgb->clear(rgb, 50);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    battery_adc.SetBitWidth(12);
    //check_battery();
    
    button_config_t btn_cfg = {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config = {
            .gpio_num = BUTTON_PIN,
            .active_level = 0,
        },
    };
    button = iot_button_create(&btn_cfg);
    iot_button_register_cb(button, BUTTON_SINGLE_CLICK, button_single_click_cb, NULL);
    iot_button_register_cb(button, BUTTON_LONG_PRESS_START, button_long_press_start_cb, NULL);
    
    // some configuration
    left_encoder.set_glitch_filter(1);
    left_encoder.start();
    right_encoder.set_glitch_filter(1);
    right_encoder.start();

    left_PID.setOutputLimits(-100, 100);
    right_PID.setOutputLimits(-100, 100);
    left_PID.setSetpoint(0);
    right_PID.setSetpoint(0);

    // tof configuration
    tof.i2cMasterInit(SDA_PIN, SCL_PIN);
    if (!tof.init()) {
        ESP_LOGE(TAG, "Failed to initialize VL53L0X :(");
        vTaskDelay(portMAX_DELAY);
    }

    // init nvs
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_LOGI(TAG, "NVS partition was truncated and needs to be erased!");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
        if (err != ESP_OK) ESP_LOGI(TAG, "NVS init still failed!");
    }
    ESP_ERROR_CHECK(err);

    // check for previous calibration values
    if (line_sensor.read_calibration_nvs() == 1)
    {
        ESP_LOGI(TAG, "No previous calibration values found!");
        set_led(100, 100, 0);
    }


    xTaskCreatePinnedToCore(
        motion_control_task,
        "Motion_control",
        10240,
        NULL,
        1,
        &mc_task_h,
        1
    );
    vTaskSuspend(mc_task_h);

    xTaskCreatePinnedToCore(
        line_follow,
        "Line_follow",
        10240,
        NULL,
        1,
        &lf_task_h,
        1
    );
    vTaskSuspend(lf_task_h);

    xTaskCreatePinnedToCore(
        read_tof_dist,
        "Read_tof",
        10240,
        NULL,
        1,
        &read_tof_h,
        0
    );
    vTaskSuspend(read_tof_h);

    while(1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}