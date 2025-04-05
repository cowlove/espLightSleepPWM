#include "jimlib.h"
#include "sensorNetworkEspNOW.h"

#include <ArduinoJson.h>

#ifndef CSIM
#include "driver/ledc.h"
#include "rom/uart.h"
#include <HTTPClient.h>
#include <esp_sleep.h>
#include <SPIFFS.h>
#define LittleFS SPIFFS
#endif

// Idea
class DeepSleepManager {
    
};

// Idea: probably don't need this, could simply look like

// bool succcss = postData(xxx)
// spiffsConsecutiveFails = success ? 0 : spiffsConsecutiveFails + 1;
// handlePostFail(spiffsConsecutiveFails) 
//
// bool success = postData(xxx);
// failRetry.setStrategy( {
//      {0,waitMin(2)}
//      {1/*fail count*/, waitMin(1)/*action*/}, 
//      {2, reboot()}, 
//      {3, waitMin(2)}, 
//      {4, waitMin{20}, 
//      {5, increaseWait(2.0)},
//      {10, sleepForever()}});
//  failRetry.reportStatus(success);
//  int newWaitSeconds = failRetry.getWaitSeconds();

namespace FailActions {
    class FailAction { public: float waitMin = 0; float multiplyWait = 1.0; bool reboot = false, halt = false; };
    struct WaitMin : public FailAction { WaitMin(float minutes) { waitMin = minutes; }};
    struct IncreaseWait : public FailAction { IncreaseWait(float mult) { multiplyWait = mult; }};
    struct HardReboot : public FailAction { HardReboot() { reboot = true; }};
    struct Halt : public FailAction { Halt() { halt = true; }};
};

class FailRetryInterval {
    string prefix;
    SPIFFSVariable<int> spiffsConsecutiveFails;
    typedef vector<pair<int, FailActions::FailAction>> FailActionList;
    FailActionList failStrategy;
    float currentWaitMin;
public:
    float defaultWaitMin;
    void setFailStrategy(FailActionList l) { failStrategy = l; };
    FailRetryInterval(const string &prefix = "", float _defaultWaitMin = 0) : 
        defaultWaitMin(_defaultWaitMin), spiffsConsecutiveFails(string("/") + prefix + "FRI.fails", 0) {}
    int reportStatus(bool success, std::function<int(int)> f = [](int fails) { return fails; }) { 
        spiffsConsecutiveFails = success ? 0 : spiffsConsecutiveFails + 1;
        return f(spiffsConsecutiveFails);
    }
    float getWaitMinutes(float defaultMin = -1) {
        if (defaultMin != -1) 
            defaultWaitMin = defaultMin;
        if (spiffsConsecutiveFails == 0) {
            currentWaitMin = defaultWaitMin;
            return currentWaitMin;
        } 
        FailActions::FailAction choice;
        for(auto i : failStrategy) {
            if (spiffsConsecutiveFails >= i.first)
                choice = i.second;
        } 
        if (choice.reboot) ESP.restart();
        if (choice.halt) { /*TODO*/}
        if (choice.waitMin != -1) currentWaitMin = choice.waitMin;
        currentWaitMin *= choice.multiplyWait;
        return currentWaitMin; 
    } 
};

void foo() {
    FailRetryInterval fr("fooRetry"/*spiff prefix*/, 2.5/*defaultMinutes*/);
    using namespace FailActions;
    fr.setFailStrategy({
        {1/*fail count*/, WaitMin(.5)}, 
        {3, IncreaseWait(10.0)},
        {5, HardReboot()},
        {10, Halt()},
    });

    while(true) { // main loop
        {
            fr.defaultWaitMin = 2.0;
            fr.reportStatus(/*success =*/true);
            float waitMin = fr.getWaitMinutes();
        }
        // or
        {
            fr.reportStatus(/*success =*/true);
            float waitMin = fr.getWaitMinutes(3.0/*defaultWait*/);
        }
    }
}


class DeepSleepElapsedTimer { // use DeepSleepElapsedTimer
    SPIFFSVariable<int64_t> bootOffsetMs;
    bool resetAfterHardReset = false;
public:
    DeepSleepElapsedTimer(const char *fn) : bootOffsetMs(fn, 0) {}
    void prepareSleep(int ms) { 
        bootOffsetMs = bootOffsetMs + ::millis() + ms;
    }
    uint32_t millis() {
        if (getResetReason(0) != 5 && resetAfterHardReset == false) { 
            resetAfterHardReset = true;
            bootOffsetMs = 0;
        }
        return (::millis()) + bootOffsetMs;
    }
    uint32_t elapsed() { return this->millis(); }
    void reset() { bootOffsetMs = -((int64_t)::millis()); }
};

DeepSleepElapsedTimer deepsleep("/deepsleep");

string getServerName() { 
    if (WiFi.SSID() == "CSIM") return "http://192.168.68.118:8080";
    if (WiFi.SSID() == "ClemmyNet") return "http://192.168.68.118:8080";
    if (WiFi.SSID() == "Station 54") return "http://192.168.68.73:8080";
    return "http://vheavy.com";
}
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

class FileLineLogger { 
    string filename;
    int lineCount = -1;
    string getNextLine(fs::File &f) { 
        string line;
        while(true) { 
            uint8_t c;
            int n = f.read(&c, 1);
            if (n != 1 || c == '\n') break;
            line += c;
        }
        return line;
    }
public:
    FileLineLogger(const string &fn) : filename(fn) {}
    void push_back(const string &s) { 
        lineCount = getLines() + 1;
        fs::File f = LittleFS.open(filename.c_str(), "a");
        f.write((uint8_t*)s.c_str(), s.length());
        f.write('\n');
    }
    string getFirstLine() { 
        fs::File f = LittleFS.open(filename.c_str(), "r");
        return getNextLine(f);
    }
    vector<string> getFirstLines(int count) {
        fs::File f = LittleFS.open(filename.c_str(), "r");
        vector<string> rval; 
        while(count-- > 0) { 
            string l = getNextLine(f);
            if(l == "") break;
            rval.push_back(l);
        }
        return rval;
    }
    void trimLinesFromFront(int count) { 
        lineCount = getLines();
        if (count == 0) {
            lineCount = 0;
            return;
        }
        vector<string> toRemove = getFirstLines(count);
        if (toRemove.size() == 0) {
            lineCount = 0;
            LittleFS.remove(filename.c_str());
            return;
        }
        int bytesToRemove = 0;
        for(auto l : toRemove) {
            bytesToRemove += l.length() + 1;
            lineCount--;
        }

        string tempfile = filename + ".T";
        fs::File f1 = LittleFS.open(filename.c_str(), "r");
        fs::File f2 = LittleFS.open(tempfile.c_str(), "w");
        f1.seek(bytesToRemove);
        while(true) {
            uint8_t buf[64];
            int n = f1.read(buf, sizeof(buf));
            if (n <= 0) break;
            f2.write(buf, n);
        }
        f1.close();
        f2.close();
        LittleFS.remove(filename.c_str());
        LittleFS.rename(tempfile.c_str(), filename.c_str());
    }
#if 0 
    void trimLinesFromFront_notruncate(int count) { 
        lineCount = getLines();
        if (count == 0) {
            lineCount = 0;
            return;
        }
        fs::File f = LittleFS.open(filename.c_str(), "r+");
        int origSize = getTotalBytes();
        vector<string> toRemove = getFirstLines(count);
        int bytesToRemove = 0;
        for(auto l : toRemove) {
            bytesToRemove += l.length() + 1;
            lineCount--;
        }
        if (bytesToRemove == origSize) { 
            f.truncate(0);
            lineCount = 0;
            return;
        }
        int pos = 0;
        while(true) { 
            uint8_t buf[64];
            f.seek(pos + bytesToRemove);
            int n = f.read(buf, sizeof(buf));
            if (n <= 0) break;
            f.seek(pos);
            f.write(buf, n);
            pos += n;
        }
        //f.truncate(origSize - bytesToRemove);
    }
#endif
    int size() { return getLines(); } 
    int getTotalBytes() {
        fs::File f = LittleFS.open(filename.c_str(), "r");
        int count = 0;
        while(true) { 
            char c;
            int n = f.read((uint8_t *)&c, 1);
            if (n != 1) break;
            count++;
        }
        return count;
    }
    int getLines() {
        if (lineCount < 0) { 
            fs::File f = LittleFS.open(filename.c_str(), "r");
            lineCount = 0;
            while(true) { 
                char c;
                int n = f.read((uint8_t *)&c, 1);
                if (n != 1) break;
                if (c == '\n') lineCount++;
            }
        }
        return lineCount;
    }
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

static inline float round(float f, float prec) { 
    return floor(f / prec + .5) * prec;
}

// TODO: avoid repeated connection attempts
class SleepyLogger { 
public:
    FileLineLogger spiffsReportLog;
    FailRetryInterval postFailTimer;
    // currently runs out of memory at about 100 at line 'vector<string> logs = spiffsReportLog' in post()
    // only succeeds after reboot when set to 90 
    static const int maxLogSize = 150; 
    DeepSleepElapsedTimer reportTimer = DeepSleepElapsedTimer("/SL.reportTimer");
    const char *TSLP = "TSLP"; // "Time Since Last Post" key/value to crease LTO "Log Time Offset" value in posted data 

    SleepyLogger(const string &prefix = "") : spiffsReportLog(prefix + "/reportLog"), postFailTimer(prefix + "/postFailTimer") {}

    void prepareSleep(int ms) {
        reportTimer.prepareSleep(ms);
    }
    JsonDocument post(JsonDocument adminDoc) {
        JsonDocument rval; 
        if (!wifiConnect()) {
            OUT("SleepyLogger connection failed");
            postFailTimer.reportStatus(false);
            return rval;
        }
        
        adminDoc["GIT"] = GIT_VERSION;
        adminDoc["MAC"] = getMacAddress().c_str(); 
        adminDoc["SSID"] = WiFi.SSID().c_str();
        adminDoc["IP"] =  WiFi.localIP().toString().c_str(); 
        adminDoc["RSSI"] = WiFi.RSSI();
        adminDoc["ARCH"] = ARDUINO_VARIANT;
        //adminDoc["AVER"] = ESP_ARDUINO_VERSION_STR;

        HTTPClient client;
        const string url = getServerName() + "/log";
        int r = client.begin(url.c_str());
        client.addHeader("Content-Type", "application/json");
        fflush(stdout);
        uart_tx_wait_idle(CONFIG_CONSOLE_UART_NUM);


        bool fail = false;
        const int batchSize = 10;
        while(spiffsReportLog.size() > 0) {
            fflush(stdout);
            uart_tx_wait_idle(CONFIG_CONSOLE_UART_NUM);
            string post;
            {   // add scope to free up admin string when we're done with it
                string admin;
                serializeJson(adminDoc, admin);
                post = "{\"ADMIN\":" + admin + ",\"LOG\":[";
            }
            int i = 0;
            bool firstLine = true;
            vector<string> lines = spiffsReportLog.getFirstLines(batchSize);
            for(auto i : lines) { 
            //for(i = 0; i < spiffsReportLog.read().size() && i < 10; i++) { 
                fflush(stdout);
                uart_tx_wait_idle(CONFIG_CONSOLE_UART_NUM);
                JsonDocument doc;
                DeserializationError error = deserializeJson(doc, i);
                if (!error && doc[TSLP].as<int>() != 0) {
                    doc["LTO"] = round((reportTimer.elapsed() - doc[TSLP].as<int>()) / 1000.0, .1)  ;
                    string s;
                    serializeJson(doc, s);
                    if (!firstLine) post += ",";
                    firstLine = false;
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
                r = client.POST(post.c_str());
                String resp =  client.getString();
                deserializeJson(rval, resp.c_str());
                OUT("http.POST returned %d: %s", r, resp.c_str());
                if (r == 200) 
                    break;
                client.end();
                client.begin(url.c_str());
                client.addHeader("Content-Type", "application/json");
            }
            if (r != 200) {  
                fail = true;
                break;
            }

            // runs out of memory here
            //vector<string> logs = spiffsReportLog;
            //logs.erase(logs.begin(), logs.begin() + i);
            //spiffsReportLog = logs;
            spiffsReportLog.trimLinesFromFront(batchSize);
        }
        client.end();

        if (fail == true) { 
            OUT("SleepyLogger repeat posts failed");
            postFailTimer.reportStatus(false);
            return rval;
        }
        postFailTimer.reportStatus(true);
        reportTimer.reset();

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
        OUT("%09.3f log() forcePost %d, reportTimer %.1f postPeriodMinutes %.1f", deepsleep.millis()/1000.0, 
        forcePost, reportTimer.elapsed()/1000.0, postFailTimer.getWaitMinutes());
        JsonDocument result; 
        doc[TSLP] = reportTimer.elapsed();
        string s;
        serializeJson(doc, s);    
        //vector<string> logs = spiffsReportLog;
        //logs.push_back(s);
        //while(logs.size() > maxLogSize)
        //    logs.erase(logs.begin());
        //spiffsReportLog = logs;
        if (spiffsReportLog.size() > maxLogSize - 1)
            spiffsReportLog.trimLinesFromFront(spiffsReportLog.size() - (maxLogSize - 1));
        spiffsReportLog.push_back(s);
        
        if (reportTimer.elapsed() > postFailTimer.getWaitMinutes() * 60 * 1000)
            forcePost = true;
        if (spiffsReportLog.size() == maxLogSize)
            forcePost = true;
        //if (LittleFS.usedBytes() > LittleFS.totalBytes() / 2 - 20 *1024)
        //    forcePost = true;
        if (forcePost) 
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

SleepyLogger logger;
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
        {3, IncreaseWait(10.0)},
        {5, HardReboot()},
        {10, Halt()},
    });

    readConfig();
    printConfig();
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
        dst["ms"] = m.asInt();
        dst["err"] = dh.temp.getRetries();
        return true;
    } 
} ambientTempSensor1("auto");

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
    doc["Tex1"] = *dht1;
    doc["Tex2"] = *dht2;
    doc["Tint"] = *dht3;
}

bool alreadyLogged = false;
uint32_t sampleStartTime = 0;
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

// TODO: observed bug where too short of a sampleTime means it never gets to log or post
// TODO: separate sensorServer sleep request and the sample loop, they are now the same
// duration 

int testMode = 0;
float vpdInt;
void loop() {
#if 0
    testLoop();
    return;
#endif
    pwm = setFan(pwm); // power keeps getting turned on???
    sensorServer.serverSleepSeconds = config.sensorTime * 60;
    sensorServer.serverSleepLinger = 12;
    int sensorWaitSec = min(10.0F, config.sampleTime * 60);
    logger.postFailTimer.defaultWaitMin = config.reportTime;

    float bv1 = hal->avgAnalogRead(pins.bv1);
    if (bv1 > 1000 && bv1 < 2000) { 
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

    // Serial log 
    if (j.secTick(1) || j.once()) {
        while(millis() < 750) delay(1); // HACK, DHT seems to need about 650ms of stable power before it can be read 
        vpdInt = getVpd(dht3);
        OUT("%09.3f Q %2d, lastpost %4.1f, snsrs seen %3d, last sns rx %3.0f, ssr %3.0f min heap %d, "
            "exvpd %.2f vpd %.2f pwm %d pow %d fs used %d/%d",
            deepsleep.millis() / 1000.0, (int)logger.spiffsReportLog.size(), 
            (int)logger.reportTimer.elapsed() / 1000.0, 
            sensorServer.countSeen(), sensorServer.lastTrafficSec(), sensorServer.getSleepRequest(),
            ESP.getMinFreeHeap(), calcVpd(ambientTempSensor1.temp.getTemperature(), ambientTempSensor1.temp.getHumidity()),
            vpdInt, pwm, hal->digitalRead(pins.power),
            LittleFS.usedBytes(), LittleFS.totalBytes()
        );
        }

    // Run PID loop and log data
    if (alreadyLogged == false && 
        ((millis() - sampleStartTime) > sensorWaitSec * 1000 || sensorServer.getSleepRequest() > 0 || forcePost)) {
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
        adminDoc["MAC"] = getMacAddress().c_str();
        adminDoc["PROG"] = basename_strip_ext(__BASE_FILE__).c_str();
        adminDoc["CONFIG"] = config;
        adminDoc["minFreeHeap"] = ESP.getMinFreeHeap();
        doc["fanPwm"] = pwm;
        doc["pidI"] = round(config.pid.iSum, .001);
        readSensors(doc);
        JsonDocument response = logger.log(doc, adminDoc, forcePost);

        if (response["CONFIG"]) {
            config.applyNewConfig(response["CONFIG"]);
            saveConfig();
        }
        OUT("%09.3f Control loop ran, vpd %.2f log queue %d, fan power %d", 
            deepsleep.millis()/1000.0, vpdInt, (int)logger.spiffsReportLog.size(), pwm);

        forcePost = false;
        alreadyLogged = true;     
    }
    
    // Decide if & how long to sleep
    if (alreadyLogged == true && millis() - sampleStartTime > config.sampleTime * 60 * 1000) { 
        // reset counter, will make another log event after
        //OUT("%09.3f Resetting sampleStartTime timer for %.2f", millis()/1000.0, config.sampleTime);
        sampleStartTime = millis();
        alreadyLogged = false;
    }
    if (hal->avgAnalogRead(pins.bv2) < config.minBatVolt) {
        OUT("Disabling fan due to battery voltage %.1f/%.1f", hal->avgAnalogRead(pins.bv2), config.minBatVolt);
        pwm = 0;
    }
    pwm = setFan(pwm);
    // should only sleep if sensorServer.getSleepRequest() is valid and > 0.  Then sleep
    // the minimum of the two sleep requests
    int sensorLoopSleepMs = sensorServer.getSleepRequest() * 1000 - 5000;
    int sampleLoopSleepMs = config.sampleTime * 60 * 1000 - (millis() - sampleStartTime);
    int sleepMs = min(sampleLoopSleepMs, sensorLoopSleepMs);


    // TODO: fix pwm light sleep issues, lightSleep() currently stubbed out with busy wait 
    if (sleepMs > 0) { 
        OUT("%09.3f sensorLoop %dms, serverLoop %dms", deepsleep.millis()/1000.0, sampleLoopSleepMs, sensorLoopSleepMs);
        bool dsleep = (pwm ==0);
#ifdef GPROF // avoid deepsleep to make one long continuous profiling run 
        deepSleep = false;
#endif
        if (dsleep) {  
            deepSleep(sleepMs);
            /* reboot */
        } else { 
            lightSleep(sleepMs);
            alreadyLogged = false;  // TODO clean up this part that resets the loop like a reboot
            sampleStartTime = millis();
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
    OUT("%09.3f DEEP SLEEP for %.2f was awake %.2fs", deepsleep.millis() / 1000.0, ms/1000.0, millis()/1000.0);
    logger.prepareSleep(ms);
    sensorServer.prepareSleep(ms);
    deepsleep.prepareSleep(ms);
    esp_sleep_enable_timer_wakeup(1000LL * ms);
    fflush(stdout);
    uart_tx_wait_idle(CONFIG_CONSOLE_UART_NUM);
    esp_deep_sleep_start();        
}

void lightSleep(int ms) { 
    OUT("%09.3f LIGHT SLEEP for %.2fs was awake %.2fs", deepsleep.millis() / 1000.0, ms/1000.0, millis()/1000.0);

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
        delay(500);
        wdtReset();
    }
    OUT("Connected to AP '%s', IP=%s, channel=%d, RSSI=%d\n",
        WiFi.SSID().c_str(), WiFi.localIP().toString().c_str(), WiFi.channel(), WiFi.RSSI());
    return WiFi.status() == WL_CONNECTED;
}

void wifiDisconnect() { 
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    j.jw.enabled = false;
}

string floatRemoveTrailingZeros(string &s) {
//#ifndef CSIM // burns up too much time in simulation
    //s = regex_replace(s, regex("[.]*[0]+}"), "}");
    //s = regex_replace(s, regex("[.]*[0]+,"), ",");
    //s = regex_replace(s, regex("[.]*[0]+]"), "]");
    s = regex_replace(s, regex("[.][0]+ "), " ");
    s = regex_replace(s, regex("[.][0]+\""), "\"");
    s = regex_replace(s, regex("[.][0]+,"), ",");
//#endif
    return s;
}

#include "RollingLeastSquares.h"
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
  RollingAverage<float, 12> intTA, intHA, extTA, extHA;
  void run(int pwm) {
    now = millis();
    if (secTick(9)) { 
        if (hal->digitalRead(pins.power)) {
            bv1 = min(2666.0L, bv1 + .000003L * speedUp);
        } else {
            bv1 = max(900.0L, bv1 - .000001L * speedUp);
        }
        float day = deepsleep.millis() / 3600000.0 / 24 * speedUp;
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
} wsim;

#ifdef CSIM
class Csim : public ESP32sim_Module {
    public:
    Csim() {
        ESPNOW_sendHandler = new ESPNOW_csimOneProg();
        csim_flags.OneProg = true;
        HTTPClient::csim_onPOST("http://.*/log", 
            [](const char *url, const char *hdr, const char *data, string &result) {
                // use csim_bootTime, log the data like a log file
                // make a real timestamp
                uint64_t nowmsec = millis() + 5LL * 365 * 24 * 3600 * 1000000 + esp32sim.bootTimeUsec / 1000;
                time_t nt = nowmsec / 1000;
                struct tm *ntm = localtime(&nt);
                char buf[64];
                //2025-03-27T03:53:24.568Z
                strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", ntm);
                printf("[%s.%03dZ] %s\n", buf, (int)((nowmsec % 1000)), data);
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
                return 200;
            });
    }
    RemoteSensorClient client1; 
    string dummy;
    void parseArg(char **&a, char **la) override {
        if (strcmp(*a, "--dummy") == 0)
                dummy = *(++a);
    }
    void setup() override {
        client1.csimOverrideMac("EC64C9986F2C");
        onDeepSleep([this](uint64_t usec) {
            client1.setPartialDeepSleep(usec);
         });
    }
    void loop() override {
        int pow = digitalRead(pins.power);
        int pwm = ESP32sim_currentPwm[2];
        wsim.run(pwm);
        
        if (dht3) 
            csim_dht.csim_set(dht3->pin, wsim.intT, wsim.intH);
        SensorDHT *sensor = (SensorDHT *)client1.findByName("TEMP");
        if (sensor) 
            csim_dht.csim_set(sensor->dht.pin, wsim.extT, wsim.extH);
        
        ESP32sim_pinManager::manager->csim_analogSet(pins.bv1, wsim.bv1); // low enough to keep csim from deep sleeping
        ESP32sim_pinManager::manager->csim_analogSet(pins.bv2, wsim.bv2);
        client1.run();
    }
} csim;
#endif

class HAL_esp32c3_HIL : public HAL {
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
        wsim.speedUp = 24.0;
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
