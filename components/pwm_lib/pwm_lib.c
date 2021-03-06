#include <stdio.h>
#include "pwm_lib.h"

// void func(void)
// {

// }

static const char* TAG = "pwmLibModule";

esp_err_t ledc_pwm_config(ledc_pwm_setup_t* pwm, bool config_timer)
{
    /*
     * Prepare and set configuration of timers
     * that will be used by LED Controller
     */
    // ledc_timer_config_t ledc_timer = {
    //     .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
    //     .freq_hz = 5000,                      // frequency of PWM signal
    //     .speed_mode = LEDC_LS_MODE,           // timer mode
    //     .timer_num = LEDC_LS_TIMER,            // timer index
    //     .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
    // };

    /*
    * Max resolution -> 2^(bits_resolution) = (APB_CLK / frequency)
    * div_param = (APB_CLK * 256) / frequency) / 2^(bit_resolution)
    * div param cannot be greater than 3FFFF (18 bit divider) and less than 256 (B part, first 8 bits)
    */
   if (config_timer)
   {
        ledc_timer_config_t ledc_timer = {
            .duty_resolution = pwm->bits_resolution,
            .freq_hz         = pwm->frequency,
            .speed_mode      = pwm->mode,
            .timer_num       = pwm->timer,
            .clk_cfg         = LEDC_AUTO_CLK
        };

        ledc_timer_config(&ledc_timer);
   }

    ledc_channel_config_t ledc_channel =
    {
        .gpio_num   = pwm->pin,
        .speed_mode = pwm->mode,
        .channel    = pwm->channel,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = pwm->timer,
        .duty       = pwm->duty_cycle,
        .hpoint     = 0,
    };

    ledc_channel_config(&ledc_channel);

    // Initialize fade service.
    // ledc_fade_func_install(0);

    pwm->max_bit_count = pow(2, pwm->bits_resolution) - 1;
    ESP_LOGI(TAG, "Max Bit Count %d", pwm->max_bit_count);

    return ESP_OK;
}

static void ledc_pwm_set_bits(ledc_pwm_setup_t* pwm, uint32_t bits)
{
    bits = (bits > pwm->max_bit_count) ? pwm->max_bit_count : bits;

    bits = (pwm->enabled) ? bits : 0; // : pwm->max_bit_count - bits;

    pwm->duty_cycle = bits;           // (bits * 100) / (pwm->max_bit_count);
    ESP_LOGI(TAG, "Duty %d", pwm->duty_cycle);
    ledc_set_duty(pwm->mode, pwm->channel, pwm->duty_cycle);
    ledc_update_duty(pwm->mode, pwm->channel);

    // ledc_set_duty_and_update()
}

void ledc_pwm_set_duty(ledc_pwm_setup_t* pwm, uint8_t duty_cycle)
{
    if (duty_cycle > 100)
        duty_cycle = 100;

    uint32_t bits = (pwm->max_bit_count * duty_cycle) / 100;
    ledc_pwm_set_bits(pwm, bits);
}

void ledc_pwm_set_frequency(ledc_pwm_setup_t* pwm, uint32_t frequency)
{
    pwm->frequency =  frequency;
    ledc_set_freq(pwm->mode, pwm->timer, pwm->frequency);
}

void ledc_pwm_stop(ledc_pwm_setup_t* pwm)
{
    pwm->enabled = false;
    ledc_stop(pwm->mode, pwm->channel, 0);
}

void ledc_pwm_resume(ledc_pwm_setup_t* pwm)
{
    pwm->enabled = true;
    ledc_update_duty(pwm->mode, pwm->channel);
}
