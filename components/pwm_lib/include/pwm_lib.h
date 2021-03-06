#pragma once

#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include <math.h>

// void func(void);

typedef struct
{
    uint8_t           pin;
    uint32_t          duty_cycle;
    ledc_mode_t       mode;
    ledc_timer_t      timer;
    ledc_channel_t    channel;
    uint32_t          frequency;
    ledc_timer_bit_t  bits_resolution;
    bool              enabled;
    uint32_t          max_bit_count;
} ledc_pwm_setup_t;

/**
 * @brief LEDC PWM configuration
 *        Configure LEDC timer with the given source timer/frequency(Hz)/duty_resolution
 *
 * @param  pwm - ledc_pwm_setup_t struct filled with all necessary data.
 * @param  config timer - bool variable to explicitly configure or re-configure timer with data in pwm struct
 *
 * @return
 *     - ESP_OK Success
 */
esp_err_t ledc_pwm_config(ledc_pwm_setup_t* pwm, bool config_timer);

void ledc_pwm_set_duty(ledc_pwm_setup_t* pwm, uint8_t duty_cycle);

void ledc_pwm_set_frequency(ledc_pwm_setup_t* pwm, uint32_t frequency);

void ledc_pwm_stop(ledc_pwm_setup_t* pwm);

void ledc_pwm_resume(ledc_pwm_setup_t* pwm);