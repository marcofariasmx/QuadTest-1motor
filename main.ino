#include "BluetoothSerial.h"
#include "esp_adc_cal.h"


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

const int ledPin = 2;  // 16 corresponds to GPIO16
const int motorPin = 26;
const int motorChannel = 1;

// setting PWM properties
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;

int pwmVal = 0;

float prevVoltRead = 0;

BluetoothSerial SerialBT;

int Led = 2;

// Potentiometer is connected to GPIO 34 (Analog ADC1_CH6) 
const int batPin = 33;

// variable for storing the potentiometer value
int batValue = 0;
float batVoltage = 0;

float R1 = 10;
float R2 = 100;

float voltDivFactor = (R1)/(R1 + R2);

float maxVoltageRead = 3.3 / voltDivFactor; // = +- 36.3

float voltResolution = maxVoltageRead / 4095;

//float correctionFactor = 1.836907;
float correctionFactor = 1.768889453;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_LABVIEW");

  // configure LED PWM functionalitites
  ledcSetup(ledChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ledPin, ledChannel);

  ledcSetup(motorChannel, freq, resolution);
  ledcAttachPin(motorPin, motorChannel);
  
}
void loop() {

  // Battery voltage readings
  
  // Reading potentiometer value
  batValue = analogRead(batPin);
  //batVoltage = map(batValue, 0.0f, 4095f, 0, 36.3); //map(value, fromLow, fromHigh, toLow, toHigh)
  batVoltage = batValue * voltResolution * correctionFactor;

  if(prevVoltRead == 0){ //initialize battery voltage to variable
    prevVoltRead = batVoltage;
  }

  prevVoltRead = ((prevVoltRead * 9) + batVoltage)/ 10;

  Serial.print(prevVoltRead);
  Serial.println(" Volts");
  
  SerialBT.print(prevVoltRead);
  SerialBT.println(" Volts");

  //Led control
  if (SerialBT.available() > 0){
    
    int pwmVal = SerialBT.parseInt();

    if (pwmVal != 0) {

      if (pwmVal == -1){
        ledcWrite(ledChannel, 0);
        ledcWrite(motorChannel, 0);
      }
      else{
        ledcWrite(ledChannel, pwmVal);
        ledcWrite(motorChannel, pwmVal);
      }
      SerialBT.println(pwmVal);
    }
  }

  delay(500);
}
