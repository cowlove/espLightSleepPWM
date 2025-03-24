#include <stdio.h>
#include <Arduino.h>
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "driver/uart.h"
#include "esp32/rom/uart.h"
#include "ulp_common.h"
#include "esp32/ulp.h"



struct LightSleepPWM { 
    static const ledc_timer_t LEDC_LS_TIMER  = LEDC_TIMER_0;
    static const ledc_mode_t LEDC_LS_MODE = LEDC_LOW_SPEED_MODE;

    int pin;
    ledc_channel_t chan;
    void ledcLightSleepSetup(int p, ledc_channel_t c) {
        pin = p;
        chan = c;

        ledc_channel_config_t ledc_channel = {
            .gpio_num = pin,
            .speed_mode = LEDC_LS_MODE,
            .channel = chan,
            .timer_sel = LEDC_LS_TIMER,
            .duty = 0,
            .hpoint = 0,
        };
        ledc_timer_config_t ledc_timer = {
            .speed_mode = LEDC_LS_MODE,          // timer mode
            .duty_resolution = LEDC_TIMER_6_BIT, // resolution of PWM duty
            .timer_num = LEDC_LS_TIMER,          // timer index
            .freq_hz = 25000,                      // frequency of PWM signal
            .clk_cfg = LEDC_USE_RTC8M_CLK,       // Force source clock to RTC8M
        };
    
        ledc_channel.gpio_num = pin;
        ledc_channel.channel = chan;

        ledc_timer_config(&ledc_timer);
        ledc_channel_config(&ledc_channel);
        //printf("Frequency %u Hz\n", ledc_get_freq(LEDC_LS_MODE, LEDC_LS_TIMER));
    }

    void ledcLightSleepSet(int i) { 
        ledc_set_duty(LEDC_LS_MODE, chan, i);
        ledc_update_duty(LEDC_LS_MODE, chan);
        esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_AUTO);
        delay(100);
        printf("Frequency %u Hz duty %d\n", 
            ledc_get_freq(LEDC_LS_MODE, LEDC_LS_TIMER),
            ledc_get_duty(LEDC_LS_MODE, chan));
    }    
} ls;

void setup() {
    Serial.begin(115200);
    ls.ledcLightSleepSetup(4, LEDC_CHANNEL_2);
}

int pwm = 10;
const int sleepSec = 5;
void loop()
{
    pwm = (pwm + 10) % 64;
    ls.ledcLightSleepSet(pwm);
    
    printf("%.3f Sleep for %ds\n", millis() / 1000.0, sleepSec);
    esp_sleep_enable_timer_wakeup(sleepSec * 1000000L);
    fflush(stdout);

    uart_tx_wait_idle(CONFIG_CONSOLE_UART_NUM);
    esp_light_sleep_start();


}

