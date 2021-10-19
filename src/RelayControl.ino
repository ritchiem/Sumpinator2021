#include <MQTT.h>
// #include <OneWire/OneWire.h>
// #include <spark-dallas-temperature/spark-dallas-temperature.h>
#include "OneWire.h"
#include <spark-dallas-temperature.h>
#include <DS18.h>

/*
 * Project RelayControl
 * Description: Adding Water depth sensor.
 * Author:
 * Date:
 */
#define DEBUG_CONSOLE true

char dev_name[32] = "";
bool configured = false;
char VERSION[64] = "2.0.1";

/*
// 2.0 - major update for first integration adding Live MQTT Publish of data, Sensors added : DHT22, Current Sensor
// 1.0 - Initial version - reading sensor and MQTT publish
*/

//Temp Buttons
const int BUT3 = D7;
const int BUT2 = D6;
const int BUT1 = D5;

// PIN ASSIGNMENT
const int RELAY1 = D4;
const int RELAY2 = D3;
const int RELAY3 = D2;

const int DHT22_PIN = D0;
const int ONE_WIRE_BUS = D1;

const int DEPTH_INPUT = A0;

void consoleLog(String out){
  if (DEBUG_CONSOLE){
    Serial.println(out);   
  }
}

int PUMP_OFF = LOW;
int PUMP_ON = HIGH;

// Tmp Butons

void setPumpStatus(int value, int pump){
      digitalWrite(pump, value);
}

void checkButton(char name, int button, int pump){

  int buttonStatus = digitalRead(button);  

  //consoleLog(String::format("Button %c Status : %d", name, buttonStatus));   

  if (buttonStatus){
    if (digitalRead(pump) == PUMP_OFF){
      publishState(String::format("button/%d",button),"1");
    }
    setPumpStatus(PUMP_ON, pump);        
  }else {    
    
    if (digitalRead(pump) == PUMP_ON){
      publishState(String::format("button/%d",button),"0");
    }

    setPumpStatus(PUMP_OFF, pump); 
  }
}

void checkButtons(){
  checkButton('1', BUT1, RELAY1);
  checkButton('2', BUT2, RELAY2);
  checkButton('3', BUT3, RELAY3);
}


// OneWire oneWire(ONE_WIRE_BUS);  // Setup a oneWire instance to communicate with any OneWire devices 
// DallasTemperature sensors(&oneWire);  // Pass our oneWire reference to Dallas Temperature.
DS18 sensor(ONE_WIRE_BUS);
uint8_t water_temp_addr[8];

// DEPTH SENSING
#define VREF_DEPTH 3470.0 // ADC's reference voltage

unsigned int readAnaloguePin(int COUNTS, int INPUT_PIN) {  
  unsigned int peakVoltage = 0;    
  bool skipRead=false;
  for (int i = 0; i < COUNTS; i++)
  {
    int rawValue = analogRead(INPUT_PIN);   //read peak voltage
    if (rawValue > 0){
      peakVoltage += rawValue;
    }else{
      Serial.print("^");
      skipRead=true;
      i--; // count agin
    }    
    delay(1);
  }

  if (skipRead){
    Serial.println("\\/");
  }

  peakVoltage = peakVoltage / COUNTS;   
  
  return peakVoltage;
}

float readAnaloguePin(int INPUT_PIN)
{
  return readAnaloguePin(5, INPUT_PIN);
}


void callback(char* topic, byte* payload, unsigned int length);
MQTT client("10.10.10.59", 1883, callback);

void callback(char* topic, byte* payload, unsigned int length) {
  char p[length + 1];
  memcpy(p, payload, length);
  p[length] = NULL;
  consoleLog(String::format("received %s",p));
}



void clientConnect(){
  consoleLog(String::format("Connecting as %s.",dev_name));
  client.connect(dev_name, "particle", "device");
}

// const int BUTTON_PIN = D1;
// const int SWITCH_PIN = D0;

void configureDevice(){

  String device_name = "unknown";

  if (strcmp("zombie_lazer", dev_name) == 0 ){
    //device_name = "zombie_lazer";
   // strlcpy(auth, "A7QWTTz27Zly-vWq9_Rnfry242Bx996Y",32); // <-- Sump Pump -- for Blynk
  }

  String json = "{ \"device_name\" : \"" + String::format("%s",dev_name) + "\""
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
                    +" }";

  // this is not good this will corrupt the subscribe as it shares buffer... but we've copied the data by this point so we're good
  Particle.publish("Sump Pump Control:", json, 60, PRIVATE);
  Particle.variable("config", json.c_str(), STRING); 
  // Not sure why passing the String in directly doesn't work but I was getting jibberish out of the API.

  clientConnect();


  configured = true;
  publishState("version", VERSION);
  
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
    Serial.begin(9600);

    //setup temp buttons
    pinMode(BUT1, INPUT_PULLDOWN);
    pinMode(BUT2, INPUT_PULLDOWN);
    pinMode(BUT3, INPUT_PULLDOWN);

    pinMode(RELAY1, OUTPUT);
    pinMode(RELAY2, OUTPUT);
    pinMode(RELAY3, OUTPUT);
 
    // Setup Temp Sensing
    // sensors.being();    

    if (sensor.read()){        
        
        sensor.addr(water_temp_addr);
    }
    

    // Setup Depth Sensing
    pinMode(DEPTH_INPUT, INPUT);

    // Configure Device
    Particle.subscribe("particle/device/name", deviceNameHandler, MY_DEVICES); // Use of MY_DEVICES as we are targeting old 1.5 API
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

void publishMQTT(String updateType, String signal,  String value) {
  if (!client.isConnected()) {
    //todo Record Client metrics
      clientConnect(); 
  }

  if (client.isConnected()){
    
    // What is this doing to munge the String
    // String::format("%s/%s/%s",dev_name, updateType, signal);
    String source = String(dev_name) +"/"+ updateType +"/"+ signal;
    consoleLog(updateType+":"+source +"="+ value);    
    client.publish(source, value);   
  }
  //todo
  //else Record Client metrics
}

void publishLive(String signal, String value){
 
  publishMQTT(LIVE, signal, value);   
}

void publishState(String signal, String value){
  publishMQTT(STATE, signal, value);   
}

void publishLiveDepth(float depth){
  consoleLog(String::format("Depth:%.2f",depth) );
  publishLive(DEPTH_SIGNAL, String::format("%.2f", depth));            
}

const String PRIMARY = "primary";
const String SECONDARY = "secondary";
void publishCurrent(String circuit, float current){
  // zombie_laser/live/primary/current/
  publishLive(String::format("%s/%s", circuit, CURRENT_SIGNAL), String::format("%.2f", current));            
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

long nextMeasureTime(){
  return millis() + 1000;
}


int reads =0 ;
int rawValue= 0;
long nextMeasure = 0; 


const float ZERO_ADJUST = 565.0; //0.4 - Voltage.
const float SIGNAL_VOLTAGE_MAX = 3.27;
const float SIGNAL_RANGE_MAX = 4096.0 - ZERO_ADJUST;

float getDepth(){
  float averageValue =  (float)rawValue / (float) reads - ZERO_ADJUST;
  float voltage = (float) averageValue * (SIGNAL_VOLTAGE_MAX / SIGNAL_RANGE_MAX);
  /// voltage 0 - 3.3v == 4-20ma == 0-5000mm      
  // Raw Value 557629 : count 1000 : average: 557.63, Voltage: 0.45, Depth: -15.02
  // Raw Value 645461 : count 1000 : average: 80.46, Voltage: 0.07, Depth: 113.94 // After sometime values fluxates .07-.08V
  float depth = voltage / SIGNAL_VOLTAGE_MAX * RANGE;

  consoleLog(String::format("Raw Value %d : count %d : average: %.2f, Voltage: %.2f, Depth: %.2f",rawValue, reads, averageValue, voltage, depth));      
  return depth;
}

void loop() { 
    checkButtons();  

    if( millis() > nextMeasure){           
      // temp sensiong
      if (sensor.read(water_temp_addr)){        
        float tempc = sensor.celsius();
        
        consoleLog(String::format("Temp: %.2fC",tempc));

        publishLive(TEMPERATURE_SIGNAL, String::format("%.2f",tempc));
      }
      
      publishLiveDepth(getDepth());



      reads = 0;
      rawValue = 0;      
      nextMeasure = millis() + 1000; //nextMeasureTime();
    }else{
      int depthRead =  analogRead(DEPTH_INPUT);
      rawValue += depthRead;
      reads++;
      //publishLiveDepth(getDepth());
    }
   
/**

    bool newLine = false;

    int rawRead = readAnaloguePin(10, DEPTH_INPUT);

    int dataVoltage = rawRead;
    float dataCurrent = ((dataVoltage / 1024.0) * VREF_DEPTH) / 120.0; //Sense Resistor:120ohm
    float depth = (dataCurrent - CURRENT_INIT) * (RANGE / DENSITY_WATER / 16.0); //Calculate depth from current readings


    if (rawRead > 0 ){      
      voltage =  rawRead/ 1024.0 * VREF_DEPTH;      
      if (newLine){
        newLine = false;
        Serial.println("!");  
      }

      Serial.print("dataCurrent:");
      Serial.print(dataCurrent);
      Serial.print("mA  : ");

      Serial.print("depth:");
      Serial.print(depth);
      Serial.print("mm  : ");

      Serial.print("voltage:");
      Serial.print(voltage);
      Serial.print("mV  Raw: ");
      Serial.print(rawRead);
      Serial.print(" ");

      current = voltage/120.0;  //Sense Resistor:120ohm  
      Serial.print("current:");
      Serial.print(current);
      Serial.println("mA");
    }else {
      Serial.print(".");
      newLine=true;
    }

    */

    // delay(1000); // better to calculate next output time rather than locking up device. 
    // i.e could be measuring average over last second 

    
}