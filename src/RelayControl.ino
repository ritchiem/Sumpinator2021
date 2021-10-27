SYSTEM_THREAD(ENABLED);

#define INTERNAL_SENSING_CODE

#include <MQTT.h>
#include "OneWire.h"
#include <spark-dallas-temperature.h>
#include <DS18.h>

#ifdef INTERNAL_SENSING_CODE

#include <PietteTech_DHT/PietteTech_DHT.h>

#endif

#include <PowerShield.h>

/*
 * Project RelayControl
 * Description: Adding Water depth sensor.
 * Author:
 * Date:
 */
#define DEBUG_CONSOLE true

char dev_name[32] = "";
bool configured = false;
char VERSION[64] = "2.5.2";

/*
// 2.6.0 - Add Software Watchdog - add System Thread(enabled)
// 2.5.2 - update failover signals to ensure errors states are cleared, remove checkbuttons
// 2.5.1 - Improved Power Failover signaling to fit HomeAssistant dashboard
// 2.5.0 - Add Wifi
// 2.4.0 - Add current 
// 2.3.0 - skip state model and just make it work. 
         - Re worked power managment.
// 2.2.0 - move to status based model
// 2.1.2 Added heartbeat
// 2.0.2. Add DHT22
// 2.0 - major update for first integration adding Live MQTT Publish of data, Sensors added : DHT22, Current Sensor<-- it wasn't coded here
// 1.0 - Initial version - reading sensor and MQTT publish
*/

#ifdef INTERNAL_SENSING_CODE
#define DHTTYPE DHT22   // DHT 22  (AM2302)
#endif

const unsigned long HEARTBEAT_INTERVAL = 5000;
const unsigned long MAIN_MEASURE_INTERVAL = 1000;

//Temp Buttons
// const int BUT2 = A4;
// const int ABUT2 = A2;
// const int ABUT3 = A3;

// PIN ASSIGNMENT

const int DC_POWER_GOOD = D6;
#define DHTPIN   D5     // Digital pin for communications
const int DHT22_PIN = DHTPIN;
const int AC_SELECTION = D4;
const int PUMP1 = D3;
const int PUMP2 = D2;
//D1  iC2 bus for Battery Monitor 
//D0 


const int ONE_WIRE_BUS = A3;
const int AC_AMP_PRIMARY_INPUT = A2;
const int AC_AMP_SECONDARY_INPUT = A1;
const int DEPTH_INPUT = A0;

bool INTERNAL_TEMP_SENSING = true;
bool DEPTH_SENSING = true;

#ifdef INTERNAL_SENSING_CODE

void dht_wrapper();                 // must be declared before the lib initialization
PietteTech_DHT DHT(DHTPIN, DHTTYPE, dht_wrapper);

// this shouldn't be needed now
void dht_wrapper() {
    DHT.isrCallback();
}

#endif

PowerShield batteryMonitor;

ApplicationWatchdog *wd;

void consoleLog(String out) {
    if (DEBUG_CONSOLE) {
        Serial.println(out);
    }
}

int PRIMARY_AC = LOW;
int SECONDARY_AC = HIGH;
int PUMP_OFF = LOW;
int PUMP_ON = HIGH;

// Tmp Butons

void setPumpStatus(int value, int pump) {
    digitalWrite(pump, value);
}

void checkButton(String name, int button, int pump) {

    int buttonStatus = digitalRead(button);

    if (buttonStatus) {
        if (digitalRead(pump) == PUMP_OFF) {
            publishState("button/" + name, "1");
        }
        setPumpStatus(PUMP_ON, pump);
    } else {

        if (digitalRead(pump) == PUMP_ON) {
            publishState("button/" + name, "0");
        }

        setPumpStatus(PUMP_OFF, pump);
    }
}

// OneWire oneWire(ONE_WIRE_BUS);  // Setup a oneWire instance to communicate with any OneWire devices 
// DallasTemperature sensors(&oneWire);  // Pass our oneWire reference to Dallas Temperature.
DS18 sensor(ONE_WIRE_BUS);
#define TEMPERATURE_PRECISION 12
uint8_t water_temp_addr[8];

// DEPTH SENSING
#define VREF_DEPTH 3470.0 // ADC's reference voltage

unsigned int readAnaloguePin(int COUNTS, int INPUT_PIN) {
    unsigned int peakVoltage = 0;
    bool skipRead = false;
    for (int i = 0; i < COUNTS; i++) {
        int rawValue = analogRead(INPUT_PIN);   //read peak voltage
        if (rawValue > 0) {
            peakVoltage += rawValue;
        } else {
            Serial.print("^");
            skipRead = true;
            i--; // count agin
        }
        delay(1);
    }

    if (skipRead) {
        Serial.println("\\/");
    }

    peakVoltage = peakVoltage / COUNTS;

    return peakVoltage;
}

float readAnaloguePin(int INPUT_PIN) {
    return readAnaloguePin(5, INPUT_PIN);
}


void callback(char *topic, byte *payload, unsigned int length);

MQTT client("10.10.10.59", 1883, callback);

void callback(char *topic, byte *payload, unsigned int length) {
    char p[length + 1];
    memcpy(p, payload, length);
    p[length] = NULL;
    consoleLog(String::format("received %s", p));
}


void clientConnect() {
    consoleLog(String::format("Connecting as %s.", dev_name));
    client.connect(dev_name, "particle", "device");
}

// const int BUTTON_PIN = D1;
// const int SWITCH_PIN = D0;

void configureDevice() {

    String device_name = "unknown";

    if (strcmp("zombie_lazer", dev_name) == 0) {
        //device_name = "zombie_lazer";
        // strlcpy(auth, "A7QWTTz27Zly-vWq9_Rnfry242Bx996Y",32); // <-- Sump Pump -- for Blynk
    }

    String json = "{ \"device_name\" : \"" + String::format("%s", dev_name) + "\""
                  // + ",\"dev_name\" : \"" + String(dev_name) + "\""
                  // + ",\"device_name_check\" : \"" + strcmp("zombie_laser", dev_name) + "\""
                  + ",\"verison\" : \"" + String(VERSION) + "\""
                  // + ",\"batteryConnected\" : " + String(batteryConnected)
                  // + ",\"rangeFinderEnabled\" : " + String(rangeFinderEnabled)
                  // + ",\"tempHumidityEnabled\" : " + String(tempHumidityEnabled)
                  // + ",\"acCurrentEnabled\" : " + String(acCurrentEnabled)
                  // + ",\"virtualPumpOnDevice\" : " + String(virtualPumpOnDevice)
                  // + ",\"batteryPublishModifier\" : " + String(batteryPublishRate)
                  // + ",\"sensorPublishFrequency\" : " + String(SAMPLE_INTERVAL)
                  + " }";

    // this is not good this will corrupt the subscribe as it shares buffer... but we've copied the data by this point so we're good
    Particle.publish("Sump Pump Control:", json, 60, PRIVATE);
    Particle.variable("config", json.c_str(), STRING);
    // Not sure why passing the String in directly doesn't work but I was getting jibberish out of the API.

    clientConnect();


    configured = true;
    publishState("version", VERSION);
    publishState("config", json);
    

    // // Connect here so we're good... hopefully.
    // if (!Blynk.connected()) {
    //   Blynk.connect();
    // }

}

void deviceNameHandler(const char *topic, const char *data) {
    strncpy(dev_name, data, sizeof(dev_name));
    configureDevice();
    Particle.unsubscribe();  // Note this will unsubscribe all handlers. Currently we only have one so no biggie.
}


// setup() runs once, when the device is first turned on.
void setup() {
    if (DEBUG_CONSOLE){
        Serial.begin(9600);
    }
    consoleLog("Device Startup");


    wd = new ApplicationWatchdog(5min, System.reset);
    
    // pinMode(ABUT2, INPUT_PULLDOWN);
    // pinMode(ABUT3, INPUT_PULLDOWN);

    // pinMode(BUT2, INPUT_PULLDOWN);

    pinMode(DC_POWER_GOOD, INPUT_PULLUP);    

    pinMode(AC_SELECTION, OUTPUT);
    pinMode(PUMP1, OUTPUT);
    pinMode(PUMP2, OUTPUT);

    setPumpStatus(PUMP_OFF, PUMP1);
    setPumpStatus(PUMP_OFF, PUMP2);
    setPumpStatus(PRIMARY_AC, AC_SELECTION);

    pinMode(DHT22_PIN, INPUT_PULLUP);

    // Setup Temp Sensing     
#ifdef INTERNAL_SENSING_CODE
    if (INTERNAL_TEMP_SENSING) {
        DHT.begin();
    }
#endif


    //External Sensor
    if (sensor.read()) {
        sensor.addr(water_temp_addr);
    }

    // Setup Depth Sensing
    pinMode(DEPTH_INPUT, INPUT);

    batteryMonitor.begin();
    batteryMonitor.quickStart();

    // Configure Device
    consoleLog("Awaiting Particle Connection");
    waitUntil(Particle.connected);
    consoleLog("Awaiting Particle Connected");
    Particle.subscribe("particle/device/name", deviceNameHandler,
                       MY_DEVICES); // Use of MY_DEVICES as we are targeting old 1.5 API
    Particle.publish("particle/device/name", PRIVATE);

}

// MQTT Signal Names
const String DEPTH_SIGNAL = "depth";
const String CURRENT_SIGNAL = "current";
const String PUMP_SIGNAL = "pump";
const String TEMPERATURE_SIGNAL = "temperature";
const String HUMIDITY_SIGNAL = "humidity";

//todo enum update types.. shared library with other clients
const String LIVE = "live";
//const String UPDATE= "update";
const String STATE = "state";

void publishMQTT(String updateType, String signal, String value) {

    String source = String(dev_name) + "/" + updateType + "/" + signal;
    consoleLog(updateType + ":" + source + "=" + value);

    if (!configured) {
        consoleLog("no MQTT, not yet configured.");
        return;
    }

    if (!client.isConnected()) {
        //todo Record Client metrics
        clientConnect();
    }

    if (client.isConnected()) {

        // What is this doing to munge the String
        // String::format("%s/%s/%s",dev_name, updateType, signal);
        client.publish(source, value);
    }
    //todo
    //else Record Client metrics
}

void publishLive(String signal, String value) {

    publishMQTT(LIVE, signal, value);
}

void publishState(String signal, String value) {
    publishMQTT(STATE, signal, value);
}

void publishLiveDepth(float depth) {
    publishLive(DEPTH_SIGNAL, String::format("%.2f", depth));
}

const String PRIMARY = "primary";
const String SECONDARY = "secondary";

void publishCurrent(String circuit, float current) {
    // zombie_laser/live/primary/current/
    publishLive(circuit + "/" + CURRENT_SIGNAL, String::format("%.2f", current));
}

int buttonStatus = false;
// loop() runs over and over again, as quickly as it can execute.

float voltage; //unit:mV
float current;  //unit:mA

#define CURRENT_INIT 0.004 // Current @ 0mm (uint: mA)
#define DENSITY_WATER 1.0  // Pure water density normalized to 1
#define DENSITY_GASOLINE 0.74  // Gasoline density
#define PRINT_INTERVAL 1000
#define RANGE 5000.0 // Depth measuring range 5000mm (for water)

int reads = 0;
int rawValue = 0;
unsigned long nextMeasure = 0;

const float ZERO_ADJUST = 565.0; //0.4 - Voltage.
const float SIGNAL_VOLTAGE_MAX = 3.27;
const float SIGNAL_RANGE_MAX = 4096.0 - ZERO_ADJUST;

float getDepth() {
    float averageValue = (float) rawValue / (float) reads - ZERO_ADJUST;
    float voltage = (float) averageValue * (SIGNAL_VOLTAGE_MAX / SIGNAL_RANGE_MAX);
    /// voltage 0 - 3.3v == 4-20ma == 0-5000mm
    // Raw Value 557629 : count 1000 : average: 557.63, Voltage: 0.45, Depth: -15.02
    // Raw Value 645461 : count 1000 : average: 80.46, Voltage: 0.07, Depth: 113.94 // After sometime values fluxates .07-.08V
    float depth = voltage / SIGNAL_VOLTAGE_MAX * RANGE;

    //consoleLog(String::format("Raw Value %d : count %d : average: %.2f, Voltage: %.2f, Depth: %.2f",rawValue, reads, averageValue, voltage, depth));
    return depth;
}

void externalTemp() {
    if (sensor.read(water_temp_addr)) {
        float tempc = sensor.celsius();
        consoleLog(String::format("Temp: %.2fC", tempc));
        publishLive(TEMPERATURE_SIGNAL, String::format("%.2f", tempc));
    }
}

bool bDHTstarted = false;                        // flag to indicate we started acquisition
void internalTemp() {
    if (!INTERNAL_TEMP_SENSING) {
        return;
    }

#ifdef INTERNAL_SENSING_CODE
    if (!DHT.acquiring()) {
        // get DHT status
        int result = DHT.getStatus();

        consoleLog("Read sensor: ");
        switch (result) {
            case DHTLIB_OK:
                consoleLog("OK");

                publishLive("internal/"+HUMIDITY_SIGNAL, String::format("%.2f", DHT.getHumidity()));
                publishLive("internal/"+TEMPERATURE_SIGNAL, String::format("%.2f", DHT.getCelsius()));

                break;
            case DHTLIB_ERROR_CHECKSUM:
                consoleLog("Error\n\r\tChecksum error");
                break;
            case DHTLIB_ERROR_ISR_TIMEOUT:
                consoleLog("Error\n\r\tISR time out error");
                break;
            case DHTLIB_ERROR_RESPONSE_TIMEOUT:
                consoleLog("Error\n\r\tResponse time out error");
                break;
            case DHTLIB_ERROR_DATA_TIMEOUT:
                consoleLog("Error\n\r\tData time out error");
                break;
            case DHTLIB_ERROR_ACQUIRING:
                consoleLog("Error\n\r\tAcquiring");
                break;
            case DHTLIB_ERROR_DELTA:
                consoleLog("Error\n\r\tDelta time to small");
                break;
            case DHTLIB_ERROR_NOTSTARTED:
                consoleLog("Error\n\r\tNot started:Restarting");
                break;
            default:
                consoleLog("Unknown error");
                break;
        }

#ifdef DEBUG_TEMP
        Serial.print("Humidity (%): ");
        Serial.println(DHT.getHumidity(), 2);

        Serial.print("Temperature (oC): ");
        Serial.println(DHT.getCelsius(), 2);

        Serial.print("Temperature (oF): ");
        Serial.println(DHT.getFahrenheit(), 2);

        Serial.print("Temperature (K): ");
        Serial.println(DHT.getKelvin(), 2);

        Serial.print("Dew Point (oC): ");
        Serial.println(DHT.getDewPoint());

        Serial.print("Dew Point Slow (oC): ");
        Serial.println(DHT.getDewPointSlow());
#endif

        bDHTstarted = false;  // reset the sample flag so we can take another
        // DHTnextSampleTime = millis() + DHT_SAMPLE_INTERVAL;  // set the time for next sample
    }
#endif

}

long heart=0;
system_tick_t nextHeartBeat;

void publishHeartbeat() {    
    if (millis() > nextHeartBeat) {
        if (heart < ULONG_MAX) {
            heart++;
        } else {
            publishLive("heartbeat/rollover", "1");
            heart = 0;
        }

        publishLive("heartbeat", String(heart));
        nextHeartBeat = millis() + HEARTBEAT_INTERVAL;
    }
}

void monitorBattery() {
    float cellVoltage = batteryMonitor.getVCell();
    float stateOfCharge = batteryMonitor.getSoC();
    

    publishLive("battery/voltage", String::format("%.2f", cellVoltage));
    publishLive("battery/soc", String::format("%.2f", stateOfCharge ));
}


void togglePowerInput() {
    int currentState = digitalRead(AC_SELECTION);
    digitalWrite(AC_SELECTION, !currentState);
}


int lastPowerStatus = -1;
int lastCircuitStatus = 0;

const int POWER_ON = 0;
const int POWER_OFF = 1; 

const int PRIMARY_CIRCUIT = 0;
const int SECONDARY_CIRCUIT = 1;

system_tick_t lastFailoverTime;
system_tick_t nextPowerMeasure;

const long FAILOVER_STEADY_STATE = 5000;
const long POWER_REPORTING_PERIOD = 1000;

int failoversSinceLastPower = 0;

const String FAILOVER_SIGNAL = "power/failover";
const String POWER_ALARM_SIGNAL = "power/alarm";

const int MAX_FAILOVERS_WITHOUT_POWER = 3600 / FAILOVER_STEADY_STATE; // 1Hr of constant failover before alarm

void publishCircuitStatus(String updateType, String state, int circuitStatus){
    switch (circuitStatus){
        case PRIMARY_CIRCUIT:
            publishMQTT(updateType, "power/"+state+"/primary", "1");
            publishMQTT(updateType, "power/"+state+"/secondary", "0");
        break;
        case SECONDARY_CIRCUIT:
            publishMQTT(updateType, "power/"+state+"/primary", "0");
            publishMQTT(updateType, "power/"+state+"/secondary", "1");
        break;
    }
}

void clearCircuitStatus(String updateType, String state){
    publishMQTT(updateType, "power/"+state+"/primary", "0");
    publishMQTT(updateType, "power/"+state+"/secondary", "0");
}
void checkMainPower() {
    int powerStatus = digitalRead(DC_POWER_GOOD); 
    int circuitStatus = digitalRead(AC_SELECTION);

    if (powerStatus == POWER_OFF){                           
        if (millis() > lastFailoverTime + FAILOVER_STEADY_STATE ){   // todo this should be extracted and use failoversSinceLastPower to backoff       

            publishLive(FAILOVER_SIGNAL, "1");
            clearCircuitStatus(STATE, "active");
            publishCircuitStatus(LIVE, "failure", circuitStatus);                  

            failoversSinceLastPower++;       
            publishLive("power/swaps", String(failoversSinceLastPower));        

            // swap power
            int circuitToActivate;
            switch (circuitStatus){
                case PRIMARY_CIRCUIT:
                circuitToActivate = SECONDARY_AC;
                break;
                case SECONDARY_CIRCUIT:
                circuitToActivate = PRIMARY_AC;
                break;
            }
            
            setPumpStatus(circuitToActivate, AC_SELECTION);
            circuitStatus = circuitToActivate;
            lastFailoverTime = millis();
        }
    }

    if (millis() > nextPowerMeasure) {        
        if (powerStatus == POWER_ON){
            publishCircuitStatus(LIVE, "active", circuitStatus);
            publishCircuitStatus(STATE, "active", circuitStatus); // not sure we need both ..state should be a persistent publish
            publishState("power/dcinput", String(powerStatus));

            //reset any failover state
            failoversSinceLastPower = 0;        
            clearCircuitStatus(LIVE, "failure");      
            publishLive(FAILOVER_SIGNAL, "0");                
            publishState(POWER_ALARM_SIGNAL, "0");            
        }        

        // // Log State Change
        if (powerStatus != lastPowerStatus){
            
            lastPowerStatus = powerStatus;           
        }

        if (circuitStatus != lastCircuitStatus){        
            lastCircuitStatus = circuitStatus;
        }

        if (failoversSinceLastPower > MAX_FAILOVERS_WITHOUT_POWER){
            publishState(POWER_ALARM_SIGNAL, "1");        
        }  

        nextPowerMeasure = millis() + POWER_REPORTING_PERIOD; 
    }
}


// lifted from https://wiki.dfrobot.com/Gravity_Analog_AC_Current_Sensor__SKU_SEN0211_
#define AC_DETECTION_RANGE 20;    //set Non-invasive AC Current Sensor tection range (5A,10A,20A)
// VREF: Analog reference
// For Arduino UNO, Leonardo and mega2560, etc. change VREF to 5
// For Arduino Zero, Due, MKR Family, ESP32, etc. 3V3 controllers, change VREF to 3.3
#define VREF 1 //Setting this to 1 looks to give better results on the Particle Xenon.

float readACCurrentValue(int ACPin)
{
    float ACCurrtntValue = 0;
    float peakVoltage = 0;  
    float voltageVirtualValue = 0;  //Vrms
    for (int i = 0; i < 5; i++)
    {
        peakVoltage += analogRead(ACPin);   //read peak voltage
        delay(1);
    }
    peakVoltage = peakVoltage / 5;   
    voltageVirtualValue = peakVoltage * 0.707;    //change the peak voltage to the Virtual Value of voltage

    /*The circuit is amplified by 2 times, so it is divided by 2.*/
    voltageVirtualValue = (voltageVirtualValue / 1024 * VREF ) / 2;  

    ACCurrtntValue = voltageVirtualValue * AC_DETECTION_RANGE;

    return ACCurrtntValue;
}

void monitorACCurrentUsage(){
    float primaryCurrent = readACCurrentValue(AC_AMP_PRIMARY_INPUT);    
    consoleLog(String::format("%.2f A",current));
    publishLive("current/primary", String::format("%.2f",primaryCurrent));

    float secondaryCurrent = readACCurrentValue(AC_AMP_SECONDARY_INPUT); 
    consoleLog(String::format("%.2f A",secondaryCurrent));        
    publishLive("current/secondary", String::format("%.2f",secondaryCurrent));
}


void monitorWifiSignal(){
    WiFiSignal signal = WiFi.RSSI();
    publishLive("wifi/strength",String::format("%.2f",signal.getStrength()));
    publishLive("wifi/quality",String::format("%.2f",signal.getQuality()));
}


void monitorSystem(){
    publishLive("system/memory",String::format("%d",System.freeMemory()));    
}

void loop() {

    //todo watchdog setup
    
    // if (configured) { // for some reason this check seems like it is being JIT'd out.. but there is no jit.
        publishHeartbeat();
        checkMainPower();        

        // MonitorPowerUsage


    // }    
    
    if (millis() > nextMeasure) {
        
        // Depth Sensor
        if (DEPTH_SENSING) {
            publishLiveDepth(getDepth());
            reads = 0;
            rawValue = 0;
        }

        // todo One-wire bus reading seems actually introduce a delay as this didn't loop un checked when the nextMeasure wasn't defined.   
        // might have been due to internal temp sensing without the .acquire() setup off measure cycle.
        // Environment Temperature Sensor. 
        externalTemp();

        monitorBattery();

        // Internal Temperature Sensor
        internalTemp();        

        monitorACCurrentUsage();

        monitorWifiSignal();

        monitorSystem();

        nextMeasure = millis() + MAIN_MEASURE_INTERVAL;        
    } else {

        if (DEPTH_SENSING) {
            //build depth average
            int depthRead = analogRead(DEPTH_INPUT);
            rawValue += depthRead;
            reads++;
        }

#ifdef INTERNAL_SENSING_CODE
        if (INTERNAL_TEMP_SENSING) {
            // ensure DHT temp measure started
            if (!bDHTstarted) {        // start the sample
                // Serial.print(": Retrieving information from sensor: ");
                DHT.acquire();
                // DHT.acquireAndWait(2000);
                bDHTstarted = true;
            }
        }
#endif

    }

    // Perform Pump control
    // }

}