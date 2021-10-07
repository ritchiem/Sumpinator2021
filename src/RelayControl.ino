/*
 * Project RelayControl
 * Description: Adding Water depth sensor.
 * Author:
 * Date:
 */
#define DEBUG_CONSOLE true

const int DEPTH_INPUT = A0;
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


// const int BUTTON_PIN = D1;
// const int SWITCH_PIN = D0;
void consoleLog(String out){
  if (DEBUG_CONSOLE){
    Serial.println(out);   
  }
}

// setup() runs once, when the device is first turned on.
void setup() {  
    Serial.begin(9600);
    pinMode(DEPTH_INPUT, INPUT);
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

void loop() { 

    if( millis() > nextMeasure){
      float averageValue =  (float)rawValue / (float) reads - ZERO_ADJUST;

      float voltage = (float) averageValue * (SIGNAL_VOLTAGE_MAX / SIGNAL_RANGE_MAX);

      /// voltage 0 - 3.3v == 4-20ma == 0-5000mm      
      // Raw Value 557629 : count 1000 : average: 557.63, Voltage: 0.45, Depth: -15.02
      // Raw Value 645461 : count 1000 : average: 80.46, Voltage: 0.07, Depth: 113.94 // After sometime values fluxates .07-.08V


      float depth = voltage / SIGNAL_VOLTAGE_MAX * RANGE;
      consoleLog(String::format("Raw Value %d : count %d : average: %.2f, Voltage: %.2f, Depth: %.2f",rawValue, reads, averageValue, voltage, depth));      
      reads = 0;
      rawValue = 0;      
      nextMeasure = millis() + 1000; //nextMeasureTime();
    }else{
       rawValue += analogRead(DEPTH_INPUT);
       reads++;
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