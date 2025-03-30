#include "jimlib.h"
#include "sensorNetworkEspNOW.h"

#include <ArduinoJson.h>

#ifndef CSIM
#include "driver/ledc.h"
#include "rom/uart.h"
#include <HTTPClient.h>
#include <esp_sleep.h>
#endif

string getServerName() { 
    if (WiFi.SSID() == "CSIM") return "http://192.168.68.118:8080";
    if (WiFi.SSID() == "ClemmyNet") return "http://192.168.68.118:8080";
    return "http://vheavy.com";
}
using std::string;
using std::vector;

JStuff j;

struct {
    int dhtGnd = 20;    // black 
    int dhtVcc = 8;     // red
    int dhtData1 = 9;   // orange
    int dhtData2 = 10;  // yellow
    int dhtData3 = 6;  // blue
    int bv1 = 2;        // purple 
    int bv2 = 3;        // purple 
    int fanPwm = 4;     // blue
    int power = 5;      // green 
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
#if SOC_PM_SUPPORT_RTC_PERIPH_PD
        esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_AUTO);
#endif
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
float calcVpd(float t, float h);
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
    static const int maxLogSize = 100;
    DeepSleepElapsedTime reportTimer;
    const char *TSLP = "TSLP"; // "Time Since Last Post" key/value to crease LTO "Log Time Offset" value in posted data 
    SleepyLogger() {
        if (reportTimer.elapsed() < 1000) 
            reportTimer.reset();
    }

    void prepareSleep(int ms) {
        reportTimer.sleep(ms);
    }
    JsonDocument post(JsonDocument adminDoc) {
        JsonDocument rval; 
        if (!wifiConnect()) {
            int sleepMin = 1;
            OUT("Failed to connect, sleeping %d min", sleepMin);
            // real light sleep, not delaySleep currently in lightSleep();
            // TODO: need to collect all deepSleep calls into a sleep manager
            // that also updates the DeepSleepElapsedTime counters like reportTimer 
            // right now they get cleared 
            digitalWrite(pins.power, 0);
            esp_sleep_enable_timer_wakeup(sleepMin * 60 * 1000000L);
            fflush(stdout);
            uart_tx_wait_idle(CONFIG_CONSOLE_UART_NUM);
            esp_light_sleep_start();
            deepSleep(0);
            return rval;
        }
        
        adminDoc["GIT"] = GIT_VERSION;
        adminDoc["MAC"] = getMacAddress().c_str(); 
        adminDoc["SSID"] = WiFi.SSID().c_str();
        adminDoc["IP"] =  WiFi.localIP().toString().c_str(); 
        adminDoc["RSSI"] = WiFi.RSSI();
        adminDoc["ARCH"] = ARDUINO_VARIANT;
        adminDoc["AVER"] = ESP_ARDUINO_VERSION_STR;

        HTTPClient client;
        const string url = getServerName() + "/log";
        int r = client.begin(url.c_str());
        OUT("http.begin() returned %d", r);
        client.addHeader("Content-Type", "application/json");

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
                client.end();
                int beginRes = client.begin(url.c_str());
                OUT("http.begin() returned %d", beginRes);
                client.addHeader("Content-Type", "application/json");
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

        if (fail == true) { 
            OUT("Failed to post, sleeping");
            deepSleep(30 * 1000);
        }
        if (reportLog.read().size() == 0) 
            reportTimer.reset();
        reportCount = reportCount + 1;

        const char *ota_ver = rval["ota_ver"];
        if (ota_ver != NULL) { 
            if (strcmp(ota_ver, GIT_VERSION) == 0 || strlen(ota_ver) == 0
                // dont update an existing -dirty unless ota_ver is also -dirty  
                //|| (strstr(GIT_VERSION, "-dirty") != NULL && strstr(ota_ver, "-dirty") == NULL)
                ) {
                OUT("OTA version '%s', local version '%s', no upgrade needed", ota_ver, GIT_VERSION);
            } else { 
                OUT("OTA version '%s', local version '%s', upgrading...", ota_ver, GIT_VERSION);
                string url = getServerName() + "/ota";
                webUpgrade(url.c_str());
            }       
        }

        wifiDisconnect();
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
        if (logs.size() > maxLogSize)
            logs.erase(logs.begin());
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
        return rval * fgain; 
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
    float vpdSetPoint;
    float minBatVolt;
    int maxFan;
    bool convertToJson(JsonVariant dst) const {
        dst["PID"] = pid;
        dst["sampleTime"] = sampleTime;
        dst["reportTime"] = reportTime;
        dst["vpdSetPoint"] = vpdSetPoint;
        dst["minBatVolt"] = minBatVolt;
        dst["maxFan"] = maxFan;
        return true;
    } 
    void convertFromJson(JsonVariantConst src) { 
        pid = src["PID"];
        vpdSetPoint = src["vpdSetPoint"] | 3.6;
        sampleTime = src["sampleTime"] | 1.0;
        reportTime = src["reportTime"] | 3.0;
        minBatVolt = src["minBatVolt"] | 1190;
        maxFan = src["maxFan"] | 20;
    }
    void applyNewConfig(const Config &c) { 
        const Config old = *this;
        *this = c;
        pid.iSum = old.pid.iSum;
        pid.lastError = old.pid.lastError;
    }
} config;

SleepyLogger logger;
DHT *dht1, *dht2, *dht3;
bool forcePost = false;
void setup() {
    j.begin();
    float bv1 = avgAnalogRead(pins.bv1);
    if (bv1 > 1000 && bv1 < 2000) { 
        printf("Battery too low %.1f sleeping 1 hour\n", bv1);
        deepSleep(60 * 60 * 1000);
    }
    pinMode(pins.dhtGnd, OUTPUT);
    digitalWrite(pins.dhtGnd, 0);
    pinMode(pins.dhtVcc, OUTPUT);
    digitalWrite(pins.dhtVcc, 1);
    dht1 = new DHT(pins.dhtData1, DHT22);
    dht2 = new DHT(pins.dhtData2, DHT22);
    dht3 = new DHT(pins.dhtData3, DHT22);
    dht1->begin();
    dht2->begin();
    dht3->begin();

    j.jw.enabled = j.mqtt.active = false;
    lsPwm.ledcLightSleepSetup(pins.fanPwm, LEDC_CHANNEL_2);
    readConfig();
    printConfig();
    OUT("quick reboots: %d", j.quickRebootCounter.reboots());
    if (getResetReason(0) != 5/*DEEP SLEEP*/)
        forcePost = true;  
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
    bool convertToJson(JsonVariant dst) const {
        RemoteSensorModuleDHT &dh = *((RemoteSensorModuleDHT *)this);
        float t = dst["t"] = round(dh.temp.getTemperature(), .01);
        float h = dst["h"] = round(dh.temp.getHumidity(), .01);
        dst["v"] = round(calcVpd(t, h), .01);
        dst["b"] = round(dh.battery.asFloat(), .01);
        return true;
    } 
} ambientTempSensor1("EC64C9986F2C");

RemoteSensorServer sensorServer({ &ambientTempSensor1 });

void yieldMs(int ms) {
    for(int i = 0; i < ms; i += 10) { 
        sensorServer.run();
        wdtReset();
        delay(10);
    }
}

template <class T>
bool convertToJson(const T &p, JsonVariant dst) { return p.convertToJson(dst); }

template <class T>
void convertFromJson(JsonVariantConst src, T &p) { p.convertFromJson(src); }

float calcVpd(float t, float rh) { 
    float sp = 0.61078 * exp((17.27 * t) / (t + 237.3)) * 7.50062;
    float vpd = (100 - rh) / 100 * sp;
    return vpd;
}

void readDht(DHT *dht, float *t, float *h) {
    *t = *h = NAN;
    wdtReset();
    //digitalWrite(pins.dhtVcc, 0);
    //yieldMs(100);
    digitalWrite(pins.dhtVcc, 1);
    //yieldMs(100);
    for(int retries = 0; retries < 10; retries++) {
        *h = dht->readHumidity();
        *t = dht->readTemperature();
        if (!isnan(*h) && !isnan(*t)) 
            break;
        retries++;
        OUT("DHT read failure %lx", dht);
        wdtReset();
        yieldMs(500);
    }
}

float getVpd(DHT *dht) { 
    float t = NAN, h = NAN;
    readDht(dht, &t, &h);
    return calcVpd(t, h);
}

bool convertToJson(const DHT &dht, JsonVariant dst) { 
    DHT *p = (DHT *)&dht;
    float t, h;
    readDht(p, &t, &h); // TMP prime DHT until we understand problems
    dst["t"] = round(t, .01);
    dst["h"] = round(h, .01);
    dst["v"] = round(calcVpd(t, h), .01);
    return true;
}

void readSensors(JsonDocument &doc) { 
    // TODO: handle stale data in Sensor::getXXX functions 
    doc["bv1"] = round(avgAnalogRead(pins.bv1), .1);
    doc["bv2"] = round(avgAnalogRead(pins.bv2), .1);
    doc["power"] = digitalRead(pins.power);
    doc["pwm"] = lsPwm.getDuty();
    doc["Tamb"] = ambientTempSensor1;
    doc["Tex1"] = *dht1;
    doc["Tex2"] = *dht2;
    doc["Tint"] = *dht3;
}

bool alreadyLogged = false;
uint32_t wakeupTime = 0;
int pwm = 0;

// TODO: observed bug where too short of a sampleTime means it never gets to log or post

int setFan(int pwm) { 
    pwm = min(config.maxFan, max(0, pwm));
    OUT("Turning on fan power level %d", pwm);
    if (pwm > 0) { 
        lsPwm.ledcLightSleepSet(pwm);
        pinMode(pins.power, OUTPUT);
        digitalWrite(pins.power, 1);
    } else { 
        pwm = 0;
        pinMode(pins.power, OUTPUT);
        digitalWrite(pins.power, 0);
        lsPwm.ledcLightSleepSet(pwm);
    }
    return pwm;
}


void testLoop() {
    #if 0 
    // trying to figure out light sleep gpio hold en
    digitalWrite(GPIO_NUM_8, 1);
    gpio_hold_en(GPIO_NUM_8);
    delay(2000);
    OUT("starting light sleep");

    esp_sleep_enable_timer_wakeup(2000 * 1000L);
    uart_tx_wait_idle(CONFIG_CONSOLE_UART_NUM);
    gpio_hold_en(GPIO_NUM_8);
    //rtc_gpio_hold_en(GPIO_NUM_8);
    gpio_deep_sleep_hold_en();
    esp_light_sleep_start();
    OUT("ending light sleep");
    wdtReset();
    return;
#endif
    static int pwm = 5; 
    j.jw.enabled = j.mqtt.active = true;
    j.run();
    if (j.secTick(1)) { 
        if(WiFi.status() != WL_CONNECTED) { 
            //wifiConnect();
        }
        OUT("RSSI: %d", WiFi.RSSI());
        setFan(pwm);
        pwm += 5;
        if (pwm > 20) pwm = 0;
        if (1) {
            JsonDocument d;
            readSensors(d);
            string s;
            serializeJson(d, s);
            OUT("%s", s.c_str());
        }
    }
}

int testMode = 0;
volatile uint32_t minFreeHeap = 0xffffffff;

uint32_t freeHeap() { 
    uint32_t fr = ESP.getFreeHeap();
    if (fr < minFreeHeap) minFreeHeap = fr;
    return fr;
}
void loop() {
#if 0
    testLoop();
    return;
#endif
    sensorServer.serverSleepSeconds = config.sampleTime * 60;
    sensorServer.serverSleepLinger = 30;
    int sensorWaitSec = 30;
    logger.reportTime = config.reportTime;
    j.run();
    //if (millis() < 5000) {
    //    delay(100);
    //    return;
    //}
    sensorServer.run();

    if (j.secTick(10) || j.once()) { 
        OUT("%09.3f queue %d, post age %d, free heap %d, min free heap %d, reset %d",
            millis() / 1000.0, (int)logger.reportLog.read().size(), 
            (int)logger.reportTimer.elapsed(), (int)freeHeap(), minFreeHeap, 
            getResetReason(0));
    }

    int sleepMs = sensorServer.getSleepRequest() * 1000;
    if (alreadyLogged == false && 
        ((millis() - wakeupTime) > sensorWaitSec * 1000 || sleepMs > 0 || forcePost)) {
        alreadyLogged = true;     
        OUT("Evaluating VPD and fan");
        float vpdInt = getVpd(dht3);
        float vpdExt = calcVpd(ambientTempSensor1.temp.getTemperature(), ambientTempSensor1.temp.getHumidity()); 
    
        if (!isnan(vpdInt) 
            && (vpdInt < config.vpdSetPoint || vpdExt < config.vpdSetPoint)) {
            float err = vpdInt - config.vpdSetPoint;
            pwm = -config.pid.calc(err);
        } else {
            pwm = 0;
        }
        pwm = setFan(pwm);
        
        JsonDocument doc, adminDoc;
        adminDoc["MAC"] = getMacAddress().c_str();
        adminDoc["PROG"] = basename_strip_ext(__BASE_FILE__).c_str();
        adminDoc["CONFIG"] = config;
        adminDoc["freeHeap"] = freeHeap();
        adminDoc["minFreeHeap"] = minFreeHeap;
        //adminDoc["LogCount"] = (int)logger.logCount;
        //adminDoc["PostCount"] = (int)logger.reportCount;

        doc["fanPwm"] = pwm;
        doc["pidI"] = round(config.pid.iSum, .001);

        readSensors(doc);
        JsonDocument response = logger.log(doc, adminDoc, forcePost);
        forcePost = false;

        if (response["CONFIG"]) {
            config.applyNewConfig(response["CONFIG"]);
            saveConfig();
        }
        OUT("%09.3f LOGGED DATA logq %d", millis() / 1000.0, (int)logger.reportLog.read().size());
    }
    
    if (alreadyLogged == true && millis() - wakeupTime > sensorServer.serverSleepSeconds * 1000) { 
        // we should have slept, never got getSensorSleepRequest(), sensors must be missing
        // reset log timer
        wakeupTime = millis();
        alreadyLogged = false;
    }
    sleepMs = sensorServer.getSleepRequest() * 1000 - 7000;
    if (sleepMs > 0) {
        if (avgAnalogRead(pins.bv1) < 2200) { // charge up our LiPo this sleep
            digitalWrite(pins.power, 1);      // TODO: pwm could still be set? 
        }
        sensorServer.prepareSleep(sleepMs); 
        if (pwm == 0 && digitalRead(pins.power) == 0) {
            logger.prepareSleep(sleepMs);
            digitalWrite(pins.power ,0);
            deepSleep(sleepMs); 
            /* reboots */
        } else { 
            digitalWrite(pins.dhtVcc, 1);
            gpio_hold_en((gpio_num_t)pins.dhtVcc);
            gpio_hold_en((gpio_num_t)pins.power);
            lightSleep(sleepMs);
            alreadyLogged = false;
            wakeupTime = millis();
        }
    }
    delay(10);
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

    // TMP can't get light sleep PWM working on ESP32C3 
    uint32_t startMs = millis();
    while(millis() - startMs < ms) { 
        delay(100);
        wdtReset();
        sensorServer.run(); // might as well as long as we're not sleeping
    }
    return;

    esp_sleep_enable_timer_wakeup(ms * 1000L);
    fflush(stdout);
    uart_tx_wait_idle(CONFIG_CONSOLE_UART_NUM);
    esp_light_sleep_start();
}

bool wifiConnect() { 
    OUT("connecting");
    wifiDisconnect(); 
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