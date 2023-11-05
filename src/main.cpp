#include <Arduino.h>
#include <EncoderButton.h>
#include <PID_v1.h>
#include <max6675.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Display.h>
#include "ADS1X15.h"
#include <WiFi.h>
#include <HTTPClient.h>

const char* ssid = "bash~ sudo rm -rf /";
const char* password = "largeroad006";

//Your Domain name with URL path or IP address with path
String serverName = "http://35.225.145.147/set";

/*
More complex espresso machine controller, orignally for Andreja (Super) Premium 
lever-converted espresso machine
*/

// Control constants:
#define PRESSURE_AVG_COUNT 1
#define PUMP_AVG_COUNT 50

// Input pins:
#define ROTARY_1 32
#define ROTARY_2 35
#define BUTTON_1 34
#define PRESSURE_INPUT_PIN 33
// PID pins:
#define BOILER_RELAY_PIN 12
#define GROUP_RELAY_PIN 14
// Pump/solenoid pins:
#define PUMP_SPEED 23
// Thermocouple breakout board pins:
#define GROUP_THERMO_SO 26
#define GROUP_THERMO_CS 25
#define GROUP_THERMO_CLK 33
#define BOILER_THERMO_SO 5
#define BOILER_THERMO_CS 18
#define BOILER_THERMO_CLK 19

double boilerSetpoint, groupSetpoint, boilerInput, groupInput, boilerOutput, groupOutput;
double pumpSetpoint, pumpInput, pumpOutput, pressureTotal, pumpTotal;
int pressureIndex, pumpIndex;
double pressures[PRESSURE_AVG_COUNT] = {};
double pumpVals[PUMP_AVG_COUNT] = {};

PID boilerPID(&boilerInput, &boilerOutput, &boilerSetpoint, 100, 50, 10, DIRECT);
PID groupPID(&groupInput, &groupOutput, &groupSetpoint, 100, 50, 10, DIRECT);
PID pumpPID(&pumpInput, &pumpOutput, &pumpSetpoint, 20, 25, 7, DIRECT);

MAX6675 groupThermo(GROUP_THERMO_CLK, GROUP_THERMO_CS, GROUP_THERMO_SO);
MAX6675 boilerThermo(BOILER_THERMO_CLK, BOILER_THERMO_CS, BOILER_THERMO_SO);

ADS1115 ADS(0x48);

EncoderButton *encoder;
Display display;

#define WINDOW_SIZE 2000  // 2 second cycle time
/* since time to calculate the multiple PID outputs is negligible compared to total window length, 
should be okay to share a single window period. */
unsigned long windowStartTime;
unsigned long last100Heartbeat;
unsigned long last1kHeartbeat;

void controlBoiler();
void controlGroup();
double readPressure();
void setOutput(double output, int relayPin);
void onEncoder(EncoderButton eb);
void onButton(EncoderButton eb);
void controlPump(double currentPressure, double targetPressure, bool set);
void getServerSettings();

void setup() {
  //Setup and serial init:
  Serial.begin(115200);

  // Wifi init
  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // set default values
  boilerSetpoint = 34;
  groupSetpoint = 30;

  // real stuff starts here:
  pinMode(BOILER_RELAY_PIN, OUTPUT);
  pinMode(GROUP_RELAY_PIN, OUTPUT);
  pinMode(PRESSURE_INPUT_PIN, INPUT);
  pinMode(PUMP_SPEED, OUTPUT);
  analogWrite(PUMP_SPEED, 0);

  windowStartTime = millis();

  // tell the PIDs to range between 0 and the full window size
  boilerPID.SetOutputLimits(0, WINDOW_SIZE);
  groupPID.SetOutputLimits(0, WINDOW_SIZE);

  // turn the PIDs on
  boilerPID.SetMode(AUTOMATIC);
  groupPID.SetMode(AUTOMATIC);
  pumpPID.SetMode(AUTOMATIC);
  pumpPID.SetSampleTime(10);

  // setup encoder
  encoder = new EncoderButton(ROTARY_1, ROTARY_2, BUTTON_1);
  encoder->setEncoderHandler(onEncoder);
  encoder->setClickHandler(onButton);

  // setup display
  display.init();
  display.P = pumpPID.GetKp();
  display.I = pumpPID.GetKi();
  display.D = pumpPID.GetKd();
  display.vals[1] = display.P;
  display.vals[2] = display.I;
  display.vals[3] = display.D;

  // setup ADC
  ADS.begin();
  ADS.setGain(1);      // 4.096 volt
  ADS.setDataRate(7);
}

void loop() {
  controlBoiler();
  controlGroup();
  encoder->update();
  display.drawMenu();
  display.pressure = readPressure();
  controlPump(max(display.pressure, 0.0), display.targetPressure, false);

  // 1s heartbeat/server check functions
  unsigned long now = millis();
  if (now - last1kHeartbeat > 2500) {
    last1kHeartbeat = now;

    getServerSettings();
  }

  // 100ms heatbeat functions
  if (now - last100Heartbeat > 100) {
    last100Heartbeat = now;

    controlPump(max(display.pressure, 0.0), display.targetPressure, true);
    pumpPID.SetTunings(display.P, display.I, display.D);
  }
}

void getServerSettings() {
  if(WiFi.status()== WL_CONNECTED){
      HTTPClient http;

      String serverPath = serverName + "?device_id=123&pressure="+display.targetPressure;
      
      // Your Domain name with URL path or IP address with path
      http.begin(serverPath.c_str());
      
      // Send HTTP GET request
      int httpResponseCode = http.GET();
      
      if (httpResponseCode==200) {
        String payload = http.getString();
        Serial.println(payload);
        
        String delim=" | ";
        int index = payload.indexOf(delim)+delim.length();
        String str = payload.substring(index, payload.indexOf(delim, index));
        double P = str.toDouble();
        
        index = payload.indexOf(delim, index)+delim.length();
        str = payload.substring(index, payload.indexOf(delim, index));
        double I = str.toDouble();
        
        index = payload.indexOf(delim, index)+delim.length();
        str = payload.substring(index, payload.indexOf(delim, index));
        double D = str.toDouble();

        // index = payload.indexOf(delim, index)+delim.length();
        // index = payload.indexOf(delim, index)+delim.length();
        // str = payload.substring(index, payload.indexOf(delim, index));
        // double targetPressure = str.toDouble();
        
        display.P = P;
        display.I = I;
        display.D = D;
        //display.targetPressure = targetPressure;

        Serial.println((String)P+" "+I+" "+D);
       
      }
      else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
      }
      // Free resources
      http.end();
    }
}

void controlBoiler() {
  boilerInput = boilerThermo.readCelsius();
  boilerPID.Compute();
  setOutput(boilerOutput, BOILER_RELAY_PIN);
}

void controlGroup() {
  groupPID.Compute();
  setOutput(groupOutput, GROUP_RELAY_PIN);
}

void setOutput(double output, int relayPin) {
  unsigned long now = millis();
  if (now - windowStartTime > WINDOW_SIZE) {  //time to shift the Relay Window
    windowStartTime += WINDOW_SIZE;
  }
  if (output > now - windowStartTime) digitalWrite(relayPin, HIGH);
  else digitalWrite(relayPin, LOW);
}

// Encoder button callback
void onButton(EncoderButton eb) {
  display.button(eb.clickCount());
}

// Encoder rotation callback
void onEncoder(EncoderButton eb) {
  display.increment(eb.increment());
}

double readPressure() {
  // analog in 0-255. Pressure transducer outputs 0.5-4.5V, for 0-300 psi (0-20.68 BarG)
  double raw = ADS.toVoltage(ADS.getValue());
  ADS.requestADC(0);
  //Serial.println((String)raw);
  //double raw = (double)analogRead(PRESSURE_INPUT_PIN);
  // convert from ADC input to PSI, accounting for ADC nonlinearity
  double psi = (raw - 0.337) * 200.0 / 4.0;

  // track moving average to reduce noise
  pressureTotal -= pressures[pressureIndex];
  pressures[pressureIndex] = psi;
  pressureTotal += pressures[pressureIndex];
  pressureIndex++;
  pressureIndex %= PRESSURE_AVG_COUNT;

  //Serial.println((String)pumpOutput + " " + (String)psi + " " + (String)(pressureTotal/((double)PRESSURE_AVG_COUNT)) + " " + (String)pressureIndex + " " + (String)pressureTotal);
  
  return pressureTotal/((double)PRESSURE_AVG_COUNT);
}

void controlPump(double currentPressure, double targetPressure, bool set) {
  pumpInput = currentPressure;
  pumpSetpoint = targetPressure;
  pumpPID.Compute();

  // track moving pump average to smooth output
  pumpTotal -= pumpVals[pumpIndex];
  pumpVals[pumpIndex] = pumpOutput;
  pumpTotal += pumpVals[pumpIndex];
  pumpIndex++;
  pumpIndex %= PUMP_AVG_COUNT;
  
  //Serial.println((String)pumpOutput + " " + (String)(pumpTotal/((double)PRESSURE_AVG_COUNT)) + " " + (String)pumpIndex + " " + (String)pumpTotal);
  
  //int output = min(max((targetPressure-currentPressure), 0.0)*10, 255.0);
  if(set) analogWrite(PUMP_SPEED, pumpTotal/((double)PUMP_AVG_COUNT));
} 