
#define INTERNAL_SENSING_CODE

#include "Particle.h"

SYSTEM_THREAD(ENABLED);
STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));

#include <MQTT.h>
#include "OneWire.h"
#include <spark-dallas-temperature.h>
#include <DS18.h>

// This package is 'said' to leak memory..
#ifdef INTERNAL_SENSING_CODE
#include <PietteTech_DHT/PietteTech_DHT.h>
#endif

#include <PowerShield.h>

/*
 * Project Sumpinator 2021
 * Description: 
 * Author: 
 * Date: 2021
 */
#define DEBUG_CONSOLE false

char dev_name[32] = "";
bool configured = false;
char VERSION[64] = "2.7.3";

/*
// 2.7.3 - Cleaned up code added Particle.Publish of heartbeet to aid in understanding if device is working.
// 2.7.2 - improve heartbeat for Home Assistant display 0/1 alternate
// 2.7.1 - message power/%s/{primary|secondary} was being truncated by 3 chars. So gave up debug and went for [32] as that is much larger than needed.
// 2.7.0 - Remove String usage as everyone says it is bad for memory and may cause lock ups.
// 2.6.1 - Correct Soc Output due to conflict with one-wire
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
const unsigned long DEPTH_MEASURE_INTERVAL = 1000;
const unsigned long MAIN_MEASURE_INTERVAL = 1000;
const unsigned long PARTICLE_UPDATE_INTERVAL = 5 * 60 * 1000;  // 5 minutes

// PIN ASSIGNMENT
const int DC_POWER_GOOD = D6;
#define DHTPIN   D5     // Digital pin for communications
const int DHT22_PIN = DHTPIN;
const int AC_SELECTION = D4;
const int PUMP1 = D3;
const int PUMP2 = D2;
//D1  iC2 bus for Battery Monitor 
//D0  used by Fuel Gauge IC2

const int ONE_WIRE_BUS = A3;
const int AC_AMP_PRIMARY_INPUT = A2;
const int AC_AMP_SECONDARY_INPUT = A1;
const int DEPTH_INPUT = A0;

bool INTERNAL_TEMP_SENSING = true;
bool DEPTH_SENSING = true;

retained long resetCounter = 0; // retained stores in SRAM so survives reset. requires above STARTUP() command to enable.

#ifdef INTERNAL_SENSING_CODE

//todo test removing wrapper apparently it isn't needed now.
void dht_wrapper();                 // must be declared before the lib initialization
PietteTech_DHT DHT(DHTPIN, DHTTYPE, dht_wrapper);

// this shouldn't be needed now
void dht_wrapper() {
    DHT.isrCallback();
}
#endif

PowerShield batteryMonitor;

ApplicationWatchdog *wd;

void consoleLog(const char* out) {
    if (DEBUG_CONSOLE) {
        Serial.println(out);
    }
}

const int PRIMARY_AC = LOW;
const int SECONDARY_AC = HIGH;
const int PUMP_OFF = LOW;
const int PUMP_ON = HIGH;

void setPumpStatus(int value, int pump) {
    digitalWrite(pump, value);
}

//todo can probably remove these onewire and dallastemp libs.
// OneWire oneWire(ONE_WIRE_BUS);  // Setup a oneWire instance to communicate with any OneWire devices 
// DallasTemperature sensors(&oneWire);  // Pass our oneWire reference to Dallas Temperature.
DS18 sensor(ONE_WIRE_BUS);
#define TEMPERATURE_PRECISION 12
uint8_t water_temperature_sensor_one_wire_address[8];


void callback(char *topic, byte *payload, unsigned int length);
MQTT client("10.10.10.59", 1883, callback); //think to fix this warning we need to create a const char* for String assignemnt and copy it in to the a char* for the API. :(

void callback(char *topic, byte *payload, unsigned int length) {
    // We don't need this call back for until we support mqtt subscription i.e. remote testing of system.

    // char p[length + 1];
    // memcpy(p, payload, length);
    // p[length] = NULL;   

    // char message[length+10];
    // snprintf(message, sizeof(message), "received %s", p);
    // consoleLog(message);
}

void clientConnect() {
    char message[sizeof(dev_name)+16];
    snprintf(message, sizeof(message), "Connecting as %s.", dev_name);
    consoleLog(message);
    client.connect(dev_name, "particle", "device");
}

void configureDevice() {
    //todo convert to char[]
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
    
    Particle.publish("Sump Pump Control:", json, 60, PRIVATE);
    Particle.variable("config", json.c_str(), STRING);
    // Not sure why passing the String in directly doesn't work but I was getting jibberish out of the API.

    clientConnect();

    configured = true;
    publishState("version", VERSION);
    publishState("config", json.c_str());
}

void deviceNameHandler(const char *topic, const char *data) {
    strncpy(dev_name, data, sizeof(dev_name));
    configureDevice();
    Particle.unsubscribe();  // Note this will unsubscribe all handlers. Currently we only have one so no biggie.
}

void mayday() {    
   resetCounter++;
   System.reset();
}

// setup() runs once, when the device is first turned on.
void setup() {
    if (DEBUG_CONSOLE){
        Serial.begin(9600);
    }
    consoleLog("Device Startup");

    wd = new ApplicationWatchdog(5min, mayday);
    
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
        sensor.addr(water_temperature_sensor_one_wire_address);
    }

    // Setup Depth Sensing
    pinMode(DEPTH_INPUT, INPUT);

    batteryMonitor.begin();
    batteryMonitor.quickStart();

    // Configure Device
    consoleLog("Awaiting Particle Connection");

    waitUntil(Particle.connected); // As we now use SYSTEM_THREAD enabled Particle connection may not be established by here.
    consoleLog("Awaiting Particle Connected");
    Particle.subscribe("particle/device/name", deviceNameHandler,
                       MY_DEVICES); // Use of MY_DEVICES as we are targeting old 1.5 API
    Particle.publish("particle/device/name", PRIVATE);
}

// MQTT Signal Names
const char* DEPTH_SIGNAL = "depth";
const char* CURRENT_SIGNAL = "current";
const char* PUMP_SIGNAL = "pump";
const char* TEMPERATURE_SIGNAL = "temperature";
const char* HUMIDITY_SIGNAL = "humidity";
const char* FAILOVER_SIGNAL = "power/failover";
const char* POWER_ALARM_SIGNAL = "power/alarm";


//todo enum update types.. shared library with other clients
const char* LIVE = "live";
//const char* UPDATE= "update";
const char* STATE = "state";

void publishMQTT(const char* updateType, const char* signal, const char* value) {
    
    char message[ sizeof(dev_name) + sizeof(updateType) + sizeof(signal) + 6];
    snprintf(message, sizeof(message), "%s/%s/%s",dev_name, updateType, signal);
    
    if (DEBUG_CONSOLE) {
        char consoleLogMessage[sizeof(message)+ sizeof(value) + 4];
        snprintf(consoleLogMessage, sizeof(consoleLogMessage), "%s = %s", message, value);    
        consoleLog(consoleLogMessage);
    }

    if (!configured) {
        consoleLog("no MQTT, not yet configured.");
        return;
    }

    if (!client.isConnected()) {
        //todo Record Client metrics
        clientConnect();
    }

    if (client.isConnected()) {
        client.publish(message, value);
    }
    //todo
    //else Record Client metrics
}

// todo would be nice to extract format copy and use a generic type for value

void publishLive(const char* signal, unsigned long value){
    const char* format = "%u";

    char message[sizeof(format) + sizeof(value) +1];
    snprintf(message, sizeof(message), format, value);

    publishMQTT(LIVE, signal, message);
}

void publishLive(const char* signal, long value){
    const char* format = "%d";

    char message[sizeof(format) + sizeof(value) +1];
    snprintf(message, sizeof(message), format, value);

    publishMQTT(LIVE, signal, message);
}

void publishLive(const char* signal, float value) {
    const char* format = "%.2f";

    char message[sizeof(format) + sizeof(value) +1];
    snprintf(message, sizeof(message), format, value);

    publishMQTT(LIVE, signal, message);
}

void publishLive(const char* signal, const char* value) {
    publishMQTT(LIVE, signal, value);
}

void publishState(const char* signal, long value){
    const char* format = "%d";

    char message[sizeof(format) + sizeof(value) +1];
    snprintf(message, sizeof(message), format, value);

    publishMQTT(STATE, signal, message);
}

void publishState(const char* signal, const char* value) {
    publishMQTT(STATE, signal, value);
}

const char* PRIMARY = "primary";
const char* SECONDARY = "secondary";
void publishCurrent(const char* circuit, float current) {

    char message[sizeof(circuit) + sizeof(CURRENT_SIGNAL) +2];
    snprintf(message, sizeof(message), "%s/%s", circuit, CURRENT_SIGNAL);

    publishLive(message, current);
}

float voltage; //unit:mV
float current;  //unit:mA

#define CURRENT_INIT 0.004 // Current @ 0mm (uint: mA)
#define DENSITY_WATER 1.0  // Pure water density normalized to 1
#define DENSITY_GASOLINE 0.74  // Gasoline density
#define PRINT_INTERVAL 1000
#define RANGE 5000.0 // Depth measuring range 5000mm (for water)

unsigned long nextMeasure = 0;

const float ZERO_ADJUST = 565.0; //0.4 - Voltage.
const float SIGNAL_VOLTAGE_MAX = 3.27;
const float SIGNAL_RANGE_MAX = 4096.0 - ZERO_ADJUST;

int reads = 0;
int rawValue = 0;
float getDepth() {
    float averageValue = (float) rawValue / (float) reads - ZERO_ADJUST;
    float voltage = (float) averageValue * (SIGNAL_VOLTAGE_MAX / SIGNAL_RANGE_MAX);
    /// voltage 0 - 3.3v == 4-20ma == 0-5000mm
    // Raw Value 557629 : count 1000 : average: 557.63, Voltage: 0.45, Depth: -15.02
    // Raw Value 645461 : count 1000 : average: 80.46, Voltage: 0.07, Depth: 113.94 // After sometime values fluxates .07-.08V
    float depth = voltage / SIGNAL_VOLTAGE_MAX * RANGE;
    
    return depth;
}

unsigned long nextDepthMeasure;
void monitorDepthSensor(){
    if (DEPTH_SENSING) {
        if (millis() > nextDepthMeasure) {                    
            publishLive(DEPTH_SIGNAL, getDepth());            
            reads = 0;
            rawValue = 0;

            nextDepthMeasure = millis() + DEPTH_MEASURE_INTERVAL;
        } else {
            //build depth average
            int depthRead = analogRead(DEPTH_INPUT);
            rawValue += depthRead;
            reads++;
        }   
    } 
}

void externalTemp() {
    if (sensor.read(water_temperature_sensor_one_wire_address)) {
        float tempc = sensor.celsius();        
        publishLive(TEMPERATURE_SIGNAL,  tempc);
    }
}


void publishLiveInternal(const char * type, float value){    
    char message[40];
    snprintf(message, sizeof(message), "internal/%s", type);
    publishLive(message, value);
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
                
                publishLiveInternal(HUMIDITY_SIGNAL, DHT.getHumidity());                
                publishLiveInternal(TEMPERATURE_SIGNAL, DHT.getCelsius());                
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

unsigned long heart = 0;
system_tick_t nextHeartBeat;

void publishHeartbeat() {    
    if (millis() > nextHeartBeat) {
        if (heart < ULONG_MAX) {
            heart++;
        } else {
            publishLive("heartbeat/rollover", "1");
            heart = 0;
        }

        publishLive("heartbeat", heart);        
        publishLive("heartbeat/pulse", heart % 2);
        
        nextHeartBeat = millis() + HEARTBEAT_INTERVAL;
    }
}

void monitorBattery() {
    float cellVoltage = batteryMonitor.getVCell();
    float stateOfCharge = batteryMonitor.getSoC();


    publishLive("battery/voltage", cellVoltage);
    publishLive("battery/soc", stateOfCharge);
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

long failoversSinceLastPower = 0;

const int MAX_FAILOVERS_WITHOUT_POWER = 3600 / FAILOVER_STEADY_STATE; // 1Hr of constant failover before alarm
void publishCircuitStatus(const char* updateType, const char* state, int circuitStatus){

    const char* primaryPattern = "power/%s/primary";
    char primaryMessage[32];    
    snprintf(primaryMessage, sizeof(primaryMessage), primaryPattern, state);

    const char* secondaryPattern = "power/%s/secondary";    
    char secondaryMessage[32];
    snprintf(secondaryMessage, sizeof(secondaryMessage), secondaryPattern, state);

    switch (circuitStatus){
        case PRIMARY_CIRCUIT:
            publishMQTT(updateType, primaryMessage, "1");
            publishMQTT(updateType, secondaryMessage, "0");
        break;
        case SECONDARY_CIRCUIT:
            publishMQTT(updateType, primaryMessage, "0");
            publishMQTT(updateType, secondaryMessage, "1");
        break;
    }
}

void clearCircuitStatus(const char* updateType, const char* state){
    const char* primaryPattern = "power/%s/primary";
    //char primaryMessage[sizeof(state)+ sizeof(primaryPattern)+ 1];
    char primaryMessage[32];
    snprintf(primaryMessage, sizeof(primaryMessage), primaryPattern, state);


    const char* secondaryPattern = "power/%s/secondary";
    //char secondaryMessage[sizeof(state)+ sizeof(secondaryPattern)+ 1];
    char secondaryMessage[32];
    snprintf(secondaryMessage, sizeof(secondaryMessage), secondaryPattern, state);

    publishMQTT(updateType, primaryMessage, "0");
    publishMQTT(updateType, secondaryMessage, "0");
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
            publishLive("power/swaps", failoversSinceLastPower);        

            // swap power
            int circuitToActivate;
            switch (circuitStatus){
                case PRIMARY_CIRCUIT:
                circuitToActivate = SECONDARY_AC;
                break;
                default: // default swap to secondary
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
            publishState("power/dcinput", powerStatus);

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
    float ACCurrentValue = 0;
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

    ACCurrentValue = voltageVirtualValue * AC_DETECTION_RANGE;

    return ACCurrentValue;
}

void monitorACCurrentUsage(){
    float primaryCurrent = readACCurrentValue(AC_AMP_PRIMARY_INPUT);       
    publishLive("current/primary", primaryCurrent);

    float secondaryCurrent = readACCurrentValue(AC_AMP_SECONDARY_INPUT); 
    publishLive("current/secondary", secondaryCurrent);
}


void monitorWifiSignal(){
    WiFiSignal signal = WiFi.RSSI();
    publishLive("wifi/strength", signal.getStrength());
    publishLive("wifi/quality", signal.getQuality());
}


void monitorSystem(){
    publishLive("system/memory", System.freeMemory());    
    publishLive("system/resetcounter", resetCounter);    
}

system_tick_t nextParticleUpdate;

void updateParticle(){
    if (millis() > nextParticleUpdate) {

        if (Particle.connected){
            char pulse[12];
            snprintf(pulse, sizeof(pulse), "%lu", heart);

            Particle.publish("heartbeat/pulse", pulse);
        }   
        
        nextParticleUpdate = millis() + PARTICLE_UPDATE_INTERVAL;
    }
}

void loop() {
   
    publishHeartbeat();    
    checkMainPower(); 
    monitorDepthSensor();

    if (millis() > nextMeasure) {
        // todo One-wire bus reading seems actually introduce a delay as this didn't loop un checked when the nextMeasure wasn't defined.   
        // might have been due to internal temp sensing without the .acquire() setup off measure cycle.
        
        // Environment Monitoring
        externalTemp();
        monitorACCurrentUsage();

        // System Monitoring
        internalTemp();        
        monitorWifiSignal();
        monitorSystem();
        monitorBattery();

        nextMeasure = millis() + MAIN_MEASURE_INTERVAL;        
    } else {
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

    // todo Create State Model 
    // Update Model on each loop
    // Check here if state is in a pumping situation    

    // update to use retained for last active pump    

    // todo also
    // MQTT subscribe so we can async update state i.e. allow for pump testing

    updateParticle();
}