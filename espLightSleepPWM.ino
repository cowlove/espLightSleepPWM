#include "jimlib.h"
#include "sensorNetworkEspNOW.h"

#include <ArduinoJson.h>

#ifndef CSIM
#include "driver/ledc.h"
#include "rom/uart.h"
#include <HTTPClient.h>
#endif

using std::string;
using std::vector;

JStuff j;

struct {
    int bv1 = 0;
    int bv2 = 0;
    int pwm = 27;
    int power = 2;
} pins;

SPIFFSVariable<string> configString("/configString3", "");

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
        //printf("Frequency %u Hz duty %d\n", 
        //    ledc_get//_freq(LEDC_LS_MODE, LEDC_LS_TIMER),
        //    ledc_get_duty(LEDC_LS_MODE, chan));
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

bool wifiConnect();
void wifiDisconnect();
void readConfig(); 
void saveConfig(); 
void printConfig(); 
void deepSleep(int ms);
void lightSleep(int ms);

// TODO: avoid repeated connection attempts
class SleepyLogger { 
public:
    SPIFFSVariable<vector<string>> reportLog = SPIFFSVariable<vector<string>>("/reportLog", {});
    SPIFFSVariable<int> reportCount = SPIFFSVariable<int>("/reportCount", 0);
    SPIFFSVariable<int> logCount = SPIFFSVariable<int>("/logCount", 0);
    SPIFFSVariable<float> reportTime = SPIFFSVariable<float>("/reportTime", 60);
    DeepSleepElapsedTime reportTimer;
    string url;

    SleepyLogger(const char *u) : url(u) {
        if (reportTimer.elapsed() < 1000) 
            reportTimer.reset();
    }

    void prepareSleep(int ms) {
        reportTimer.sleep(ms);
    }
    JsonDocument post() {
        JsonDocument rval; 
        if (!wifiConnect())
            return rval;

        HTTPClient client;
        int r = client.begin(url.c_str());
        OUT("http.begin() returned %d", r);
        client.addHeader("Content-Type", "application/json");
        
        bool fail = false;
        while(reportLog.read().size() > 0) { 
            JsonDocument doc;
            DeserializationError error = deserializeJson(doc, reportLog.read()[0]);

            if (!error && doc["TSL"].as<int>() != 0) {
                doc["LogTimeOffsetSec"] = (reportTimer.elapsed() - doc["TSL"].as<int>()) / 1000.0;
                string s;
                serializeJson(doc, s);
                Serial.printf("POST '%s'\n", s.c_str());
                for(int retry = 0; retry < 5; retry ++) {
                    wdtReset(); 
                    r = client.POST(s.c_str());
                    String resp =  client.getString();
                    deserializeJson(rval, resp.c_str());
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

        if (reportLog.read().size() == 0) 
            reportTimer.reset();
        reportCount = reportCount + 1;
        return rval;
    }

    JsonDocument log(JsonDocument doc, bool forcePost = false) {
        JsonDocument result; 
        logCount = logCount + 1;
        doc["TSL"] = reportTimer.elapsed();
        string s;
        serializeJson(doc, s);    
        vector<string> logs = reportLog;
        logs.push_back(s);
        reportLog = logs;
        if (forcePost == true || reportTimer.elapsed() > reportTime * 60 * 1000)
            result = post();
        return result;
    }
};


class SimplePID { 
public:
    float pgain = 1, igain = 1, dgain = 1, fgain = 10;
    float lastError = -1, iSum = 0;
    bool operator ==(const SimplePID &b) { 
        return memcmp((void *)this, (void *)&b, sizeof(*this)) == 0;
    }
    float calc(float err) {
        iSum += err;
        float rval = pgain * err + igain * iSum + dgain * (lastError - err);
        lastError = err;
        return rval; 
     }
    bool convertToJson(JsonVariant dst) const { 
        char buf[128];
        snprintf(buf, sizeof(buf), "P=%f I=%f D=%f F=%f L=%f S=%f", 
            pgain, igain, dgain, fgain, lastError, iSum);
        return dst.set(buf);
    }
    void convertFromJson(JsonVariantConst src) { 
        if(src.as<const char *>() != NULL)
            sscanf(src.as<const char *>(), "P=%f I=%f D=%f F=%f L=%f S=%f", 
                &pgain, &igain, &dgain, &fgain, &lastError, &iSum);
    }
};

template <class T>
bool convertToJson(const T &p, JsonVariant dst) { return p.convertToJson(dst); }

template <class T>
void convertFromJson(JsonVariantConst src, T &p) { p.convertFromJson(src); }

struct Config {
    SimplePID pid;
    float sampleTime;
    float reportTime;
    bool convertToJson(JsonVariant dst) const {
        dst["PID"] = pid;
        dst["SampleTime"] = sampleTime;
        dst["reportTime"] = reportTime;
        return true;
    } 
    void convertFromJson(JsonVariantConst src) { 
        pid = src["PID"];
        sampleTime = src["SampleTime"] | 0.2;
        reportTime = src["reportTime"] | 3.0;
    }
    void applyNewConfig(const Config &c) { 
        const Config old = *this;
        *this = c;
        pid.iSum = old.pid.iSum;
        pid.lastError = old.pid.lastError;
    }
} config;



#ifdef CSIM
const char *url = "http://localhost:8080/log";
#else 
const char *url = "http://192.168.68.118:8080/log";
#endif
SleepyLogger logger(url);

bool forcePost = false;
void setup() {
    j.begin();
    j.jw.enabled = j.mqtt.active = false;
    ls.ledcLightSleepSetup(pins.pwm, LEDC_CHANNEL_2);
    readConfig();
    printConfig();
    OUT("quick reboots: %d", j.quickRebootCounter.reboots());
    if (j.quickRebootCounter.reboots() > 2) 
        forcePost = true;
    OUT("RESET REASON: %s", reset_reason_string(getResetReason()));
}

class RemoteSensorModuleDHT : public RemoteSensorModule {
    public:
        RemoteSensorModuleDHT(const char *mac) : RemoteSensorModule(mac) {}
        SensorOutput gnd = SensorOutput(this, "GND", 27, 0);
        SensorDHT temp = SensorDHT(this, "TEMP", 26);
        SensorOutput vcc = SensorOutput(this, "VCC", 25, 1);
        SensorADC battery = SensorADC(this, "LIPOBATTERY", 33, .0017);
        SensorOutput led = SensorOutput(this, "LED", 22, 0);
        SensorVariable v = SensorVariable(this, "RETRY", "X10");
        SensorMillis m = SensorMillis(this);
        ////} ambientTempSensor1("EC64C9986F2C");
} ambientTempSensor1("auto");
    
RemoteSensorServer sensorServer({ &ambientTempSensor1 });

void readSensors(JsonDocument &doc) { 
    doc["Voltage1"] = avgAnalogRead(pins.bv1);
    doc["Voltage2"] = avgAnalogRead(pins.bv2);
    // TODO: handle stale data in Sensor::getXXX functions 
    float temp = NAN, hum = NAN;
    if (ambientTempSensor1.temp.getAgeMs() < sensorServer.serverSleepSeconds * 1000) {
        temp = ambientTempSensor1.temp.getTemperature();
        hum = ambientTempSensor1.temp.getHumidity();
    }
    doc["Temp"] = temp; 
    doc["Hum"] = hum;
    doc["TempAgeSec"] = ambientTempSensor1.temp.getAgeMs() / 1000.0;
}

int pwm = 1;
bool alreadyLogged = false;
uint32_t wakeupTime = 0;

void loop() {
    sensorServer.serverSleepSeconds = config.sampleTime * 60;
    sensorServer.serverSleepLinger = 30;
    int sensorWaitSec = 30;
    logger.reportTime = config.reportTime;

    j.run();
    sensorServer.run();
    if (!j.secTick(1)) { 
        delay(1);
        return;
    }   

    if (j.secTick(10)) { 
        OUT("%09.3f logq %d, %d since post, free heap %d",
            millis() / 1000.0,  (int)logger.reportLog.read().size(), 
            (int)logger.reportTimer.elapsed(), (int)ESP.getFreeHeap());
    }
    int sleepMs = sensorServer.getSleepRequest() * 1000;
    if (alreadyLogged == false && 
        ((millis() - wakeupTime) > sensorWaitSec * 1000 || sleepMs > 0)) {
        alreadyLogged = true;     
        ls.ledcLightSleepSet(pwm);
        pinMode(pins.power, OUTPUT);
        digitalWrite(pins.power, 1);
        pwm = (pwm + 5) % 64;
        
        float vpd = 0.0;
        float fanPwm = config.pid.calc(vpd);
        
        JsonDocument doc;
        doc["MAC"] = getMacAddress().c_str();
        doc["PROGRAM"] = basename_strip_ext(__BASE_FILE__).c_str();
        doc["PWM"] = fanPwm; 
        doc["CONFIG"] = config;
        doc["LogCount"] = (int)logger.logCount;
        doc["PostCount"] = (int)logger.reportCount;
        readSensors(doc);
        JsonDocument response = logger.log(doc, forcePost);
        forcePost = false;

        if (response["CONFIG"]) {
            config.applyNewConfig(response["CONFIG"]);
            saveConfig();
        }
        OUT("%09.3f LOGGED DATA logq %d", millis() / 1000.0, (int)logger.reportLog.read().size());
    }
    
    if (millis() - wakeupTime > sensorServer.serverSleepSeconds * 1000) { 
        // we should have slept, never got getSensorSleepRequest(), sensors must be missing
        // reset log timer
        wakeupTime = millis();
    }
    if (sleepMs > 0) {
        sensorServer.prepareSleep(sleepMs); 
        if (millis() > 8 * 60 * 1000) {
            logger.prepareSleep(sleepMs);
            deepSleep(sleepMs); 
            /* reboots */
        } else { 
            lightSleep(sleepMs);
            alreadyLogged = false;
            wakeupTime = millis();
        }
    }
}

void readConfig() { 
    JsonDocument doc;
    deserializeJson(doc, configString.read());
    config = doc["CONFIG"];
}

void printConfig() { 
    JsonDocument d2;
    string s;
    d2["CONFIG"] = config;
    serializeJson(d2, s);
    Serial.println(s.c_str());
}

void saveConfig() { 
    JsonDocument d2;
    string s;
    d2["CONFIG"] = config;
    serializeJson(d2, s);
    configString = s;
}

void deepSleep(int ms) { 
    OUT("%09.3f DEEP SLEEP for %dms", millis() / 1000.0, ms);
    esp_sleep_enable_timer_wakeup(1000LL * ms);
    fflush(stdout);
    uart_tx_wait_idle(CONFIG_CONSOLE_UART_NUM);
    esp_deep_sleep_start();        
}

void lightSleep(int ms) { 
    OUT("%09.3f LIGHT SLEEP for %dms", millis() / 1000.0, ms);
    esp_sleep_enable_timer_wakeup(ms * 1000L);
    fflush(stdout);
    uart_tx_wait_idle(CONFIG_CONSOLE_UART_NUM);
    esp_light_sleep_start();
}

bool wifiConnect() { 
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
    return WiFi.status() == WL_CONNECTED;
}

void wifiDisconnect() { 
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    j.jw.enabled = false;
}


