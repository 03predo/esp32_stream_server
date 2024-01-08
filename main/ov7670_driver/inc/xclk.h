#pragma once

#include "esp_system.h"
#include "driver/ledc.h"

esp_err_t xclk_timer_conf(int ledc_timer, int xclk_freq_hz);

esp_err_t camera_enable_out_clock(ledc_timer_t ledc_timer, ledc_channel_t ledc_channel, int xclk_pin, int xclk_freq_hz);

void camera_disable_out_clock(void);
