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

#include "jimlib.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>

using std::string;
using std::vector;

JStuff j;

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


class DeepSleepElapsedTime {
    SPIFFSVariable<int> bootStartMs = SPIFFSVariable<int>("/deepSleepElapsedTime", 0);
public:
    DeepSleepElapsedTime() {}
    void sleep(int msToSleep) { bootStartMs = bootStartMs + millis() + msToSleep; } 
    int elapsed() { return bootStartMs + millis(); }
    void reset() { bootStartMs = -((int)millis()); }
};

DeepSleepElapsedTime reportTimer;

template<> bool fromString(const string &s, std::vector<string> &v) {
    v = split(s, '\n');
    return true;
}

template<> string toString(const std::vector<string> &v) { 
    string rval;
    for(auto line : v) rval += line + "\n";
    return rval;
}

SPIFFSVariable<vector<string>> reportLog("/reportLog", {});

void logReport() { 
    String mac = getMacAddress();
    string s = 
        sfmt("{\"PROGRAM\":\"%s\",", basename_strip_ext(__BASE_FILE__).c_str()) + 
        sfmt("\"TSL\":%d,", (int)reportTimer.elapsed()) + 
        sfmt("\"GIT_VERSION\":\"%s\",", GIT_VERSION) + 
        sfmt("\"MAC\":\"%s\",", mac.c_str()) + 
        sfmt("\"SSID\":\"%s\",", WiFi.SSID().c_str()) + 
        sfmt("\"IP\":\"%s\",", WiFi.localIP().toString().c_str()) + 
        sfmt("\"RSSI\":%d}\n", WiFi.RSSI());

    vector<string> logs = reportLog;
    logs.push_back(s);
    reportLog = logs;
}

void wifiConnect() { 
    OUT("connecting"); 
    j.jw.enabled = true;
    j.mqtt.active = false;
    j.jw.onConnect([](){});
    j.jw.autoConnect();
    for(int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) { 
        delay(1000);
        wdtReset();
    }
    OUT("Connected to AP '%s' in %dms, IP=%s, channel=%d, RSSI=%d\n",
        WiFi.SSID().c_str(), millis(), WiFi.localIP().toString().c_str(), WiFi.channel(), WiFi.RSSI());
}

void wifiDisconnect() { 
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    j.jw.enabled = false;
}

void postReport() { 
    wifiConnect();
    if (WiFi.status() != WL_CONNECTED)
        return;

    HTTPClient client;
    int r = client.begin("http://vheavy.com/log");
    OUT("http.begin() returned %d", r);
    client.addHeader("Content-Type", "application/json");
    
    bool fail = false;
    while(reportLog.get().size() > 0) { 
        string s = reportLog.get()[0];
        if (s.length() > 0) {  
            Serial.printf("POST '%s'\n", s.c_str());
            for(int retry = 0; retry < 5; retry ++) {
                wdtReset(); 
                r = client.POST(s.c_str());
                String resp =  client.getString();
                OUT("http.POST returned %d: %s", r, resp.c_str());
                if (r == 200) 
                    break;
            }
            if (r != 200) {  
                fail = true;
                break;
            }
        }
        vector<string> logs = reportLog;
        logs.erase(logs.begin());
        reportLog = logs;
    }
    client.end();
    wifiDisconnect();

    if (reportLog.get().size() == 0) 
        reportTimer.reset();
}

void deepSleep(int ms) { 
    OUT("%09.3f DEEP SLEEP for %dms", millis() / 1000.0, ms);
    esp_sleep_enable_timer_wakeup(1000LL * ms);
    fflush(stdout);
    uart_tx_wait_idle(CONFIG_CONSOLE_UART_NUM);
    reportTimer.sleep(ms);
    esp_deep_sleep_start();        
}

void lightSleep(int ms) { 
    OUT("%09.3f LIGHT SLEEP for %ds", millis() / 1000.0, ms);
    esp_sleep_enable_timer_wakeup(ms * 1000L);
    fflush(stdout);
    uart_tx_wait_idle(CONFIG_CONSOLE_UART_NUM);
    esp_light_sleep_start();
}

struct { 
    int pwm = 26;
    int power = 2;
} pins;

void setup() {
    j.begin();
    j.jw.enabled = j.mqtt.active = false;
    Serial.begin(115200);
    ls.ledcLightSleepSetup(pins.pwm, LEDC_CHANNEL_2);
}

int pwm = 1;
const int sleepSec = 10;
void loop() {
    j.run();
    OUT("%09.3f WiFi IP: %s", millis() / 1000.0, WiFi.localIP().toString().c_str());

    ls.ledcLightSleepSet(pwm);
    pinMode(pins.power, OUTPUT);
    digitalWrite(pins.power, 1);
    pwm = (pwm + 5) % 64;
    delay(1000);

    logReport();
    if (reportTimer.elapsed() > 60 * 1000) 
        postReport();

    if (j.secTick(130)) {
        deepSleep(sleepSec * 1000); 
    }
    lightSleep(sleepSec * 1000);
}

