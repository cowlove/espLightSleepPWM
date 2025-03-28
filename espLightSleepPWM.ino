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
    int dhtGnd = 5; // TMP
    int dhtData1 = 6;
    int dhtVcc = 7; // TMP
    int bv1 = 2;
    int bv2 = 3; 
    int power = 9;
    int fanPwm = 8;
    int dhtData2 = 0; // TODO
    int dhtData3 = 0; // TODO 
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
    int getDuty() { return ledc_get_duty(LEDC_LS_MODE, chan); }     
} lsPwm;

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
string floatRemoveTrailingZeros(string &);

float round(float f, float prec) { 
    return floor(f / prec + .5) * prec;
}

// TODO: avoid repeated connection attempts
class SleepyLogger { 
public:
    SPIFFSVariable<vector<string>> reportLog = SPIFFSVariable<vector<string>>("/reportLog", {});
    SPIFFSVariable<int> reportCount = SPIFFSVariable<int>("/reportCount", 0);
    SPIFFSVariable<int> logCount = SPIFFSVariable<int>("/logCount", 0);
    SPIFFSVariable<float> reportTime = SPIFFSVariable<float>("/reportTime", 60);
    DeepSleepElapsedTime reportTimer;
    string url;
    const char *TSLP = "TSLP"; // "Time Since Last Post" key/value to crease LTO "Log Time Offset" value in posted data 
    SleepyLogger(const char *u) : url(u) {
        if (reportTimer.elapsed() < 1000) 
            reportTimer.reset();
    }

    void prepareSleep(int ms) {
        reportTimer.sleep(ms);
    }
    JsonDocument post(JsonDocument adminDoc) {
        JsonDocument rval; 
        if (!wifiConnect())
            return rval;

        HTTPClient client;
        int r = client.begin(url.c_str());
        OUT("http.begin() returned %d", r);
        client.addHeader("Content-Type", "application/json");
        
        adminDoc["GIT"] = GIT_VERSION;
        adminDoc["MAC"] = getMacAddress().c_str(); 
        adminDoc["SSID"] = WiFi.SSID().c_str();
        adminDoc["IP"] =  WiFi.localIP().toString().c_str(); 
        adminDoc["RSSI"] = WiFi.RSSI();
        adminDoc["ARCH"] = ARDUINO_VARIANT;

        bool fail = false;
        while(reportLog.read().size() > 0) {
            string admin, data, post;
            serializeJson(adminDoc, admin);
            post = "{\"ADMIN\":" + admin + ",\"LOG\":[";

            int i = 0;
            for(i = 0; i < reportLog.read().size() && i < 10; i++) { 
                JsonDocument doc;
                DeserializationError error = deserializeJson(doc, reportLog.read()[i]);
                if (!error && doc[TSLP].as<int>() != 0) {
                    doc["LTO"] = round((reportTimer.elapsed() - doc[TSLP].as<int>()) / 1000.0, .1)  ;
                    string s;
                    serializeJson(doc, s);
                    if (i != 0) post += ",";
                    post += s;
                } 
            }
            post += "]}";
            post = floatRemoveTrailingZeros(post);

            // reserialize the digest and remove TSLP values 
            JsonDocument tmp;
            DeserializationError error = deserializeJson(tmp, post);
            JsonArray a = tmp["LOG"].as<JsonArray>();
            for(JsonVariant v : a) v.remove(TSLP);
            serializeJson(tmp, post);

            for(int retry = 0; retry < 5; retry ++) {
                wdtReset(); 
                OUT("POST: %s", post.c_str());
                r = client.POST(post.c_str());
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

            vector<string> logs = reportLog;
            logs.erase(logs.begin(), logs.begin() + i);
            reportLog = logs;
        }
        client.end();
        wifiDisconnect();

        if (reportLog.read().size() == 0) 
            reportTimer.reset();
        reportCount = reportCount + 1;
        return rval;
    }

    JsonDocument log(JsonDocument doc, JsonDocument adminDoc, bool forcePost = false) {
        JsonDocument result; 
        logCount = logCount + 1;
        doc[TSLP] = reportTimer.elapsed();
        string s;
        serializeJson(doc, s);    
        vector<string> logs = reportLog;
        logs.push_back(s);
        reportLog = logs;
        if (forcePost == true || reportTimer.elapsed() > reportTime * 60 * 1000)
            result = post(adminDoc);
        return result;
    }
};


class SimplePID { 
public:
    float pgain = 1, igain = 1, dgain = 1, fgain = 10, maxI = 10;
    float lastError = -1, iSum = 0;
    bool operator ==(const SimplePID &b) { 
        return memcmp((void *)this, (void *)&b, sizeof(*this)) == 0;
    }
    float calc(float err) {
        iSum += err;
        iSum = max(-maxI, min(maxI, iSum));
        float rval = pgain * err + igain * iSum + dgain * (lastError - err);
        lastError = err;
        return rval; 
     }
    bool convertToJson(JsonVariant dst) const { 
        char buf[128];
        snprintf(buf, sizeof(buf), "P=%f I=%f D=%f F=%f L=%f S=%f MI=%f", 
            pgain, igain, dgain, fgain, lastError, iSum, maxI);
        return dst.set(buf);
    }
    void convertFromJson(JsonVariantConst src) { 
        if(src.as<const char *>() != NULL)
            sscanf(src.as<const char *>(), "P=%f I=%f D=%f F=%f L=%f S=%f MI=%f", 
                &pgain, &igain, &dgain, &fgain, &lastError, &iSum, &maxI);
    }
};

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
        sampleTime = src["SampleTime"] | 1.0;
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
DHT *dht1 = NULL;
bool forcePost = false;
void setup() {
    pinMode(pins.dhtGnd, OUTPUT);
    digitalWrite(pins.dhtGnd, 0);
    pinMode(pins.dhtVcc, OUTPUT);
    digitalWrite(pins.dhtVcc, 1);

    dht1 = new DHT(pins.dhtData1, DHT22);
    dht1->begin();

    j.begin();
    j.jw.enabled = j.mqtt.active = false;
    lsPwm.ledcLightSleepSetup(pins.fanPwm, LEDC_CHANNEL_2);
    readConfig();
    printConfig();
    OUT("quick reboots: %d", j.quickRebootCounter.reboots());
    if (j.quickRebootCounter.reboots() > 2 || getResetReason(0) == 1) 
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
    SensorMillis m = SensorMillis(this);
} ambientTempSensor1("auto");

RemoteSensorServer sensorServer({ &ambientTempSensor1 });

bool convertToJson(const RemoteSensorModuleDHT &t, JsonVariant dst) {
    dst["temp"] = t.temp.getTemperature();
    dst["hum"] = t.temp.getHumidity();
    //dst["age"] = t.temp.getAgeMs();
    dst["bat"] = round(t.battery.asFloat(), .01);
    return true;
} 
template <class T>
bool convertToJson(const T &p, JsonVariant dst) { return p.convertToJson(dst); }

template <class T>
void convertFromJson(JsonVariantConst src, T &p) { p.convertFromJson(src); }



void readDht(DHT *dht, float *t, float *h) {
    *t = *h = NAN;
    for(int retries = 0; retries < 10; retries++) {
        *t = dht->readTemperature();
        *h = dht->readHumidity();
        if (!isnan(*h) && !isnan(*t)) 
            break;
        retries++;
        OUT("DHT read failure");
        j.run();
        delay(500);
    }
}

RemoteSensorModuleDHT x("auto");

bool convertToJson(const DHT &dht, JsonVariant dst) { 
    DHT *p = (DHT *)&dht;
    dst["temp"] = p->readTemperature();
    dst["hum"] = p->readHumidity();
    return true;
}


void readSensors(JsonDocument &doc) { 
    // TODO: handle stale data in Sensor::getXXX functions 
    doc["bv1"] = round(avgAnalogRead(pins.bv1), .1);
    doc["bv2"] = round(avgAnalogRead(pins.bv2), .1);
    doc["power"] = digitalRead(pins.power);
    doc["pwm"] = lsPwm.getDuty();
    //doc["RDHT1"] = ambientTempSensor1;
    doc["LDHT1"] = *dht1;
}

int pwm = 1;
bool alreadyLogged = false;
uint32_t wakeupTime = 0;


// TODO: observed bug where too short of a sampleTime means it never gets to log or post

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
        ((millis() - wakeupTime) > sensorWaitSec * 1000 || sleepMs > 0 || forcePost)) {
        alreadyLogged = true;     

        lsPwm.ledcLightSleepSet(pwm);
        pinMode(pins.power, OUTPUT);
        digitalWrite(pins.power, 1);
        pwm = (pwm + 5) % 64;
        
        float vpd = 0.0;
        float fanPwm = config.pid.calc(vpd);
        
        JsonDocument doc, adminDoc;
        adminDoc["MAC"] = getMacAddress().c_str();
        adminDoc["PROG"] = basename_strip_ext(__BASE_FILE__).c_str();
        adminDoc["CONFIG"] = config;
        adminDoc["LogCount"] = (int)logger.logCount;
        adminDoc["PostCount"] = (int)logger.reportCount;

        doc["fanPwm"] = fanPwm; 
        readSensors(doc);
        JsonDocument response = logger.log(doc, adminDoc, forcePost);
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
        alreadyLogged = false;
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

string floatRemoveTrailingZeros(string &s) {
    s = regex_replace(s, regex("[.]*[0]+ "), " ");
    s = regex_replace(s, regex("[.]*[0]+\""), "\"");
    s = regex_replace(s, regex("[.]*[0]+$"), "");
    return s;
}