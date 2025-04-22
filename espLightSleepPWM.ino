#ifndef CSIM
//#include "driver/ledc.h"
//#include "rom/uart.h"
#include <HTTPClient.h>
#include <esp_sleep.h>
#include <LittleFS.h>
//#include <SPIFFS.h>
//#define LittleFS SPIFFS
#endif
//#include <ArduinoJson.h>

#include "jimlib.h"
#include "batchWebLogger.h"
#include "sensorNetworkEspNOW.h"
#include "serialLog.h"
#include "RollingLeastSquares.h"
#include "simulatedFailures.h"

JStuff j;
	
string getServerName() { 
    if (WiFi.SSID() == "CSIM") return "http://192.168.68.118:8080";
    if (WiFi.SSID() == "ClemmyNet") return "http://192.168.68.118:8080";
    if (WiFi.SSID() == "Station 72") return "http://192.168.86.26:8080";
    return "http://vheavy.com";
}
using std::string;
using std::vector;


//                     ---USB PLUG---
//   bv1    purp       2           +5V      red     power
//   bv2    purp       3           GND      blk     ground
//   fan    blue       4   TOP     +3.3V
//   fanpow green      5           10       yellow  dht2
//   dht3   blue       6           9        orange  dht1
//   NC                7           8        red     dhtvcc
//   NC                21          20       blk     dhtgnd 

#if defined(ARDUINO_ESP32C3_DEV) || defined(CSIM) || defined(ARDUINO_ESP32S3_DEV)
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
#elif defined(ARDUINO_ESP32_DEV)
struct {
    int dhtGnd = 25;    // black 
    int dhtVcc = 27;     // red
    int dhtData1 = 26;   // orange
    int dhtData2 = 26;  // yellow
    int dhtData3 = 26;  // blue
    int bv1 = 33;        // purple 
    int bv2 = 35;        // purple 
    int fanPwm = 12;     // blue
    int power = 13;      // green 
} pins;
#else
#error Unsupported board
#endif


void yieldMs(int);

class HAL {
public:
    LightSleepPWM lsPwm;
    virtual float avgAnalogRead(int p) { return ::avgAnalogRead(p); }
    virtual void readDht(DHT *dht, float *t, float *h) {
        uint32_t startMs = millis();
        *t = *h = NAN;
        wdtReset();
        //::digitalWrite(pins.dhtVcc, 0);
        //yieldMs(100);
        //::digitalWrite(pins.dhtVcc, 1);
        //yieldMs(2500);
        for(int retries = 0; retries < 10; retries++) {
            *h = dht->readHumidity();
            *t = dht->readTemperature();
            if (!isnan(*h) && !isnan(*t)) 
                break;
            retries++;
            OUT("DHT %lx read failure", dht);
            wdtReset();
            yieldMs(1000);
        }
        //OUT("DHT %lx read %.2f %.2f in %dms", dht, *t, *h, millis() - startMs);    
    }
    virtual void digitalWrite(int p, int v) { ::digitalWrite(p, v); }
    virtual int digitalRead(int p) { return ::digitalRead(p); }
    virtual void pinMode(int p, int mode) { ::pinMode(p, mode);}    
    virtual void setPWM(int p, int pwm) {
        lsPwm.ledcLightSleepSetup(p, LEDC_CHANNEL_2);
        lsPwm.ledcLightSleepSet(pwm);
    }    
} halHW;

HAL *hal = &halHW;
void setHITL();
    
SPIFFSVariable<string> configString("/configString3", "");
DeepSleepElapsedTimer realtimeMs("/deepsleep");

//bool wifiConnect();
//void wifiDisconnect();
void readConfig(); 
void saveConfig(); 
void printConfig(); 
void lightSleep(int); 
float calcVpd(float t, float h);
string floatRemoveTrailingZeros(string &);

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
    float sensorTime;
    float reportTime;
    float vpdSetPoint;
    float minBatVolt;
    int maxFan;
    bool convertToJson(JsonVariant dst) const {
        dst["PID"] = pid;
        dst["sampleTime"] = sampleTime;
        dst["sensorTime"] = sensorTime;
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
        sensorTime = src["sensorTime"] | 3.0;
        reportTime = src["reportTime"] | 5.0;
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

BatchWebLogger logger;
DHT *dht1, *dht2, *dht3;
bool forcePost = false;

void setup() {
    hal->pinMode(pins.dhtGnd, OUTPUT);
    hal->digitalWrite(pins.dhtGnd, 0);
    hal->pinMode(pins.dhtVcc, OUTPUT);
    hal->digitalWrite(pins.dhtVcc, 1);

    if (getMacAddress() == "E4B323C55708") setHITL();
    if (getMacAddress() == "A0DD6C725F8C") setHITL();
    if (getMacAddress() == "08F9E0F6E0B0") setHITL();
    if (getMacAddress() == "F0F5BD723D08") setHITL();
    if (getMacAddress() == "CCBA9716E0D8") setHITL();
    if (getMacAddress() == "0CB815C2412C") setHITL();

    
    j.begin();
    dht1 = new DHT(pins.dhtData1, DHT22);
    dht2 = new DHT(pins.dhtData2, DHT22);
    dht3 = new DHT(pins.dhtData3, DHT22);
    dht1->begin();
    dht2->begin();
    dht3->begin();
    j.jw.enabled = j.mqtt.active = false;
    
    using namespace FailActions;
    logger.postFailTimer.setFailStrategy({
        {1/*fail count*/, WaitMin(.5)}, 
        {3, WaitMin(5)},
        {5, IncreaseWait(2)},
        {10, Callback([](){
            OUT("Too many failures, formatting flash and resetting");
            LittleFS.format();
            ESP.restart();
        })},
        {20, MultiplyWait(3)},
        {22, HardReboot()},
       // {10, Halt()},
    });
    logger.getServerName = getServerName;

    readConfig();
    printConfig();
    if (getResetReason(0) != 5/*DEEP SLEEP*/) { 
        forcePost = true;  
        logger.postFailTimer.reset();
    }
    dsm().onDeepSleep([](int){ saveConfig(); });
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
            dst["ms"] = m.asInt();
            dst["err"] = dh.temp.getRetries();
            return true;
        } 
    } ambientTempSensor1("auto");
    
class RemoteSensorModuleScale : public RemoteSensorModule {
public:
    RemoteSensorModuleScale(const char *mac) : RemoteSensorModule(mac) {}
    SensorOutput gnd = SensorOutput(this, "GND", 27, 0);
    SensorDHT temp = SensorDHT(this, "TEMP", 26);
    SensorOutput vcc = SensorOutput(this, "VCC", 25, 1);
    SensorADC battery = SensorADC(this, "LIPOBATTERY", 33, .0017);
    SensorOutput led = SensorOutput(this, "LED", 22, 0);
    SensorMillis m = SensorMillis(this);
    SensorOutput gnd2 = SensorOutput(this, "GND", 22, 0);
    SensorOutput vcc2 = SensorOutput(this, "VCC", 19, 1);
    SensorHX711 weight = SensorHX711(this, "WEIGHT", 23, 18);
    bool convertToJson(JsonVariant dst) const {
        RemoteSensorModuleScale &dh = *((RemoteSensorModuleScale *)this);
        float t = dst["t"] = round(dh.temp.getTemperature(), .01);
        float h = dst["h"] = round(dh.temp.getHumidity(), .01);
        dst["v"] = round(calcVpd(t, h), .01);
        dst["b"] = round(dh.battery.asFloat(), .01);
        dst["ms"] = m.asInt();
        dst["err"] = dh.temp.getRetries();
        dst["weight"] = dh.weight.get();
        return true;
    } 
} scaleSensor1("F8B3B77BBE20");

RemoteSensorServer sensorServer({ &ambientTempSensor1, &scaleSensor1 });

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

float getVpd(DHT *dht) { 
    float t = NAN, h = NAN;
    hal->readDht(dht, &t, &h);
    return calcVpd(t, h);
}

bool convertToJson(const DHT &dht, JsonVariant dst) { 
    DHT *p = (DHT *)&dht;
    float t, h;
    hal->readDht(p, &t, &h); // TMP prime DHT until we understand problems
    dst["t"] = round(t, .01);
    dst["h"] = round(h, .01);
    dst["v"] = round(calcVpd(t, h), .01);
    return true;
}

void readSensors(JsonDocument &doc) { 
    // TODO: handle stale data in Sensor::getXXX functions 
    doc["bv1"] = round(hal->avgAnalogRead(pins.bv1), .1);
    doc["bv2"] = round(hal->avgAnalogRead(pins.bv2), .1);
    doc["power"] = hal->digitalRead(pins.power);
    doc["Tamb"] = ambientTempSensor1;
    doc["scale"] = scaleSensor1;  
    doc["Tex1"] = *dht1;
    doc["Tex2"] = *dht2;
    doc["Tint"] = *dht3;
}

bool alreadyLogged = false;
uint32_t sampleStartTs = 0;
int pwm = 0;

int setFan(int pwm) { 
    pwm = min(config.maxFan, max(0, pwm));
    //OUT("Turning on fan power level %d", pwm);
    int power = pwm > 0;
    hal->pinMode(pins.power, OUTPUT);
    hal->digitalWrite(pins.power, power);
    hal->setPWM(pins.fanPwm, pwm); 
    return pwm;
}

// TODO: observed bug where too short of a sampleTime means it never gets to log or post
// TODO: separate sensorServer sleep request and the sample loop, they are now the same
// duration 

int testMode = 0;
float vpdInt;
void loop() {
#if 0
    for (int n = 0; n < 5; n++) { 
        int x = hal->avgAnalogRead(pins.bv1);
        int y = 0;//adc1_get_raw(ADC1_CHANNEL_1);
        printf("analogRead(%d) %d %d\n", pins.bv1, x, y);
        delay(100);
    }
    wdtReset();
    return;
#endif
    pwm = setFan(pwm); // power keeps getting turned on???
    sensorServer.synchPeriodMin = config.sensorTime;

    int sensorWaitSec = min(10.0F, config.sampleTime * 60);
    logger.postFailTimer.defaultWaitMin = config.reportTime;

    float bv1 = hal->avgAnalogRead(pins.bv1);
    if (false && bv1 > 1000 && bv1 < 2000) { 
        printf("Battery too low %.1f sleeping 1 hour\n", bv1);
        int sleepMs = 60 * 60 * 1000;
        deepSleep(sleepMs);
    }

    j.run();
    // errors connecting to WIFI too early? 
    //if (millis() < 5000) {
    //    delay(100);
    //    return;
    //}
    sensorServer.run();

    // Make a serial log entry
    if (j.secTick(2) || j.once()) {
        vpdInt = getVpd(dht3);
        OUT("QQ %d nextl %.0f nextp %.0f snsrs %d lastsnsr %.0f ssr %.0f "
            "ev %.1f iv %.1f bv1 %.1f pwm %d pow %d fs %d/%d heap %d,%d",
            (int)logger.spiffsReportLog.size(), 
            config.sampleTime * 60 - (millis() - sampleStartTs) / 1000.0,
            logger.postFailTimer.getWaitMinutes() * 60 - logger.postPeriodTimer.elapsed() / 1000.0, 
            sensorServer.countSeen(), sensorServer.lastTrafficSec(), sensorServer.getSleepRequest(),
            calcVpd(ambientTempSensor1.temp.getTemperature(), ambientTempSensor1.temp.getHumidity()),
            vpdInt, hal->avgAnalogRead(pins.bv1), pwm, hal->digitalRead(pins.power),
            LittleFS.usedBytes(), LittleFS.totalBytes(), ESP.getMinFreeHeap(),
            ESP.getFreeHeap()
        );
    }

    // Run PID loop and log data.  We log after waiting sensorWaitSec every time we wake up, whether sleep or not.
    // Thereafter, log every config.sampleTime, corrdinated by resetting alreadyLogged and sampleStartTime below.
    if (alreadyLogged == false && 
        ((millis() - sampleStartTs) > sensorWaitSec * 1000 || sensorServer.getSleepRequest() > 0 || forcePost)) {
        //  TODO: || seems like an error ... &&?             ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        float vpdExt = calcVpd(ambientTempSensor1.temp.getTemperature(), ambientTempSensor1.temp.getHumidity()); 
        if (!isnan(vpdInt) 
            && (vpdInt < config.vpdSetPoint || vpdExt < config.vpdSetPoint)) {
            float err = vpdInt - config.vpdSetPoint;
            if (config.pid.iSum > 0) config.pid.iSum = 0; // TMP limit iSum to negative until we use a plant model 
            pwm = -config.pid.calc(err);
        } else {
            pwm = 0;
        }
        pwm = setFan(pwm);

        JsonDocument doc, adminDoc;
        String mac = getMacAddress(), base = basename_strip_ext(__BASE_FILE__);
        adminDoc["MAC"] = mac.c_str();
        adminDoc["PROG"] = base.c_str();
        adminDoc["CONFIG"] = config;
        adminDoc["minFreeHeap"] = ESP.getMinFreeHeap();
        doc["fanPwm"] = pwm;
        doc["pidI"] = round(config.pid.iSum, .001);
        readSensors(doc);
        bool inhibitPost = sensorServer.getSleepRequest() < 20.0;
        JsonDocument response = logger.log(doc, adminDoc, forcePost, inhibitPost);

        if (response["CONFIG"]) {
            //OUT("Got new config in post respose");
            config.applyNewConfig(response["CONFIG"]);
            saveConfig();
        }
        //OUT("%09.3f Control loop ran, vpd %.2f log queue %d, fan power %d", 
        //    deepsleepMs.millis()/1000.0, vpdInt, (int)logger.spiffsReportLog.size(), pwm);

        forcePost = false;
        alreadyLogged = true;     
    }
    
    // Decide if & how long to sleep
    if (alreadyLogged == true && millis() - sampleStartTs > config.sampleTime * 60 * 1000) { 
        // reset counter, will make another log event after
        //OUT("%09.3f Resetting sampleStartTime timer for %.2f", millis()/1000.0, config.sampleTime);
        sampleStartTs = millis();
        alreadyLogged = false;
    }
    if (hal->avgAnalogRead(pins.bv2) < config.minBatVolt) {
        OUT("Disabling fan due to battery voltage %.1f/%.1f", hal->avgAnalogRead(pins.bv2), config.minBatVolt);
        pwm = 0;
    } else if (bv1 < 2400 && bv1 > 1000 && pwm == 0) {
        pwm = 1; // minimal pwm setting to charge battery 
    }
    pwm = setFan(pwm);
    // should only sleep if sensorServer.getSleepRequest() is valid and > 0.  Then sleep
    // the minimum of the two sleep requests
    int sensorLoopSleepMs = sensorServer.getSleepRequest() * 1000;
    int sampleLoopSleepMs = config.sampleTime * 60 * 1000 - (millis() - sampleStartTs);
    int sleepMs = min(sampleLoopSleepMs, sensorLoopSleepMs);


    //OUT("bv1: %f", hal->avgAnalogRead(pins.bv1));
    // TODO: fix pwm light sleep issues, lightSleep() currently stubbed out with busy wait 
    if (sleepMs > 0) { 
        //OUT("%09.3f sensorLoop %dms, serverLoop %dms", deepsleepMs.millis()/1000.0, sampleLoopSleepMs, sensorLoopSleepMs);
        bool dsleep = (pwm == 0);
#ifdef GPROF // avoid deepsleep to make one long continuous profiling run 
        dsleep = false;
#endif
        if (dsleep) {  
            deepSleep(sleepMs);
            /* reboot */
        } else { 
            lightSleep(sleepMs);
            alreadyLogged = false;  // TODO clean up this part that resets the loop like a reboot
            sampleStartTs = millis();
            vpdInt = getVpd(dht3);
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
    JsonDocument doc;
    doc["CONFIG"] = config;
    string s;
    serializeJson(doc, s);
    floatRemoveTrailingZeros(s);
    OUT("%s\n", s.c_str());
}

void saveConfig() { 
    JsonDocument d2;
    string s;
    d2["CONFIG"] = config;
    serializeJson(d2, s);
    configString = s;
}

void lightSleep(int ms) { 
    OUT("LIGHT SLEEP for %.2fs was awake %.2fs", ms/1000.0, millis()/1000.0);
#ifndef CSIM
    // TMP can't get light sleep PWM working on ESP32C3 
    uint32_t startMs = millis();
    while(millis() - startMs < ms) { 
        delay(100);
        wdtReset();
        sensorServer.run(); // might as well as long as we're not sleeping
    }
    return;
#endif
    esp_sleep_enable_timer_wakeup(ms * 1000L);
    fflush(stdout);
    uart_tx_wait_idle(CONFIG_CONSOLE_UART_NUM);
    esp_light_sleep_start();
}


class WorldSim {
public:
  long double bv1 = 2100, bv2 = 1500, intT = 0, intH = 0, extT = 0, extH = 0;
  float speedUp = 1.0;
  uint32_t lastRun = millis(), now = millis();
  bool firstLoop = true;
  bool secTick(float sec) {
    if (firstLoop == true) { 
        firstLoop = false;
        return true;
    } 
    return floor(now / (int)(sec * 1000)) != floor(lastRun / (int)(sec * 1000));
  }
  RollingAverage<float, 8> intTA, intHA, extTA, extHA;
  void run(int pwm) {
    now = millis();
    if (secTick(8)) { 
        if (hal->digitalRead(pins.power)) {
            bv1 = min(2666.0L, bv1 + .000003L * speedUp);
        } else {
            bv1 = max(900.0L, bv1 - .000001L * speedUp);
        }
        float day = realtimeMs.millis() / 3600000.0 / 24 * speedUp;
        intTA.add(max(2.0, cos(day * 2 * M_PI) * 30 - 4) + pwm * 0.3);
        intT = intTA.average();
        extTA.add(max(2.0, cos(day * 2 * M_PI) * 35 - 2));
        extT = extTA.average();
        intHA.add(max(0.0L, 90.0 - intT * 1.6 - pwm * 3.5));
        extHA.add(max(0.0L, 90.0 - extT * 1.6));
        intH = intHA.average();
        extH = extHA.average();
    }
    lastRun = millis(); 
  }  
};

#ifdef CSIM
class CsimSketch : public Csim_Module {
    WorldSim wsim;
public:
    CsimSketch() {
        ESPNOW_sendHandler = new ESPNOW_csimOneProg();
    }
    RemoteSensorClient client1; 
    string dummy;
    void parseArg(char **&a, char **la) override {
        if (strcmp(*a, "--dummy") == 0)
                dummy = *(++a);
    }
    void setup() override {
        csim_flags.OneProg = true;
        HTTPClient::csim_onPOST("http://.*/log", 
            [](const char *url, const char *hdr, const char *data, string &result) {
                JsonDocument doc;
                Config simConfig = doc["CONFIG"]; // get defaults 
                simConfig.sensorTime = 30;
                simConfig.sampleTime = 1;
                simConfig.reportTime = 100;
                simConfig.pid.fgain = 2;
                doc["ota_ver"] = "";
                doc["status"] = 1;
                doc["CONFIG"] = simConfig;
                result = "";
                serializeJson(doc, result);
                if (SIMFAILURE("http-post")) 
                    return -123;
                return 200;
            });
        client1.csimOverrideMac("EC64C9986F2C");
        csim_onDeepSleep([this](uint64_t usec) {
            client1.prepareSleep(usec / 1000);
        });
        //WiFi.simulatedFailMinutes = 1;
    }
    void loop() override {
        int pow = digitalRead(pins.power);
        int pwm = Csim_currentPwm[2];
        wsim.run(pwm);
        
        if (dht3) 
            DHT::csim_set(dht3->pin, wsim.intT, wsim.intH);
        SensorDHT *sensor = (SensorDHT *)client1.findByName("TEMP");
        if (sensor) 
            DHT::csim_set(sensor->dht.pin, wsim.extT, wsim.extH);
        
        Csim_pins().csim_analogSet(pins.bv1, wsim.bv1); // low enough to keep csim from deep sleeping
        Csim_pins().csim_analogSet(pins.bv2, wsim.bv2);
        client1.run();
    }
} csim;
#endif

class HAL_esp32c3_HIL : public HAL {
    WorldSim wsim;
    typedef HAL Parent; 
public:
    float avgAnalogRead(int p) override {
        if (p == pins.bv1) return wsim.bv1;
        if (p == pins.bv2) return wsim.bv2;
        return Parent::avgAnalogRead(p);
    };
    void readDht(DHT *d, float *t, float *h) override {
        if (0) { // hack for HW test setup with only 1 real DHT-
            Parent::readDht(dht1, t, h);
            return; 
        }

        struct BackdoorDHT { 
            uint8_t data[5];
            uint8_t _pin, _type;
        };
        BackdoorDHT *dht = (BackdoorDHT *)d;
        wsim.speedUp = 24.0 * 60 / 10; // day every 10 minutes
        wsim.run(lsPwm.getDuty());
        if (dht->_pin == pins.dhtData3) {
            *t = wsim.intT;
            *h = wsim.intH;
        }
    };
} halHIL;

void setHITL() { 
    hal = &halHIL;
}
