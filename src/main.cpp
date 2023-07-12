#include <Arduino.h>
#include <EncoderButton.h>
#include <PID_v1.h>
#include <max6675.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Display.h>

/*
More complex espresso machine controller, orignally for Andreja (Super) Premium 
lever-converted espresso machine
*/

// Input pins:
#define ROTARY_1 32
#define ROTARY_2 35
#define BUTTON_1 34
#define PRESSURE_INPUT_PIN 33
#define PRESSURE_AVG_COUNT 100
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
double pumpSetpoint, pumpInput, pumpOutput,pressureTotal;
int pressureIndex;
double pressures[PRESSURE_AVG_COUNT] = {};

PID boilerPID(&boilerInput, &boilerOutput, &boilerSetpoint, 100, 50, 10, DIRECT);
PID groupPID(&groupInput, &groupOutput, &groupSetpoint, 100, 50, 10, DIRECT);
PID pumpPID(&pumpInput, &pumpOutput, &pumpSetpoint, 100, 0, 0, DIRECT);

MAX6675 groupThermo(GROUP_THERMO_CLK, GROUP_THERMO_CS, GROUP_THERMO_SO);
MAX6675 boilerThermo(BOILER_THERMO_CLK, BOILER_THERMO_CS, BOILER_THERMO_SO);

EncoderButton *encoder;
Display display;

#define WINDOW_SIZE 2000  // 2 second cycle time
/* since time to calculate the multiple PID outputs is negligible compared to total window length, 
should be okay to share a single window period. */
unsigned long windowStartTime;
unsigned long lastHeartbeat;

void controlBoiler();
void controlGroup();
double readPressure();
void setOutput(double output, int relayPin);
void onEncoder(EncoderButton eb);
void onButton(EncoderButton eb);
void controlPump(double currentPressure, double targetPressure);

void setup() {
  //Setup and serial init:
  Serial.begin(115200);

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

  // setup encoder
  encoder = new EncoderButton(ROTARY_1, ROTARY_2, BUTTON_1);
  encoder->setEncoderHandler(onEncoder);
  encoder->setClickHandler(onButton);

  // setup display
  display.init();
}

void loop() {
  controlBoiler();
  controlGroup();
  encoder->update();
  display.drawMenu();
  display.pressure = readPressure();


  unsigned long now = millis();
  if (now - lastHeartbeat > 250) {
    controlPump(display.pressure, (double)encoder->position());
    Serial.println((String)pumpOutput + " " + (String)readPressure() + " " + (String)pressureIndex + " " + (String)pressureTotal);
    lastHeartbeat = now;
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
  /************************************************
     turn the given SSR pin on/off based on pid output
   ************************************************/
  unsigned long now = millis();
  if (now - windowStartTime > WINDOW_SIZE) {  //time to shift the Relay Window
    windowStartTime += WINDOW_SIZE;
  }
  if (output > now - windowStartTime) digitalWrite(relayPin, HIGH);
  else digitalWrite(relayPin, LOW);
}

// Encoder button callback
void onButton(EncoderButton eb) {
  Serial.print("button clickCount: ");
  Serial.println(eb.clickCount());

  display.button(eb.clickCount());
}

// Encoder rotation callback
void onEncoder(EncoderButton eb) {
  Serial.print("encoder position is: ");
  Serial.println(eb.position());

  display.increment(eb.increment());
}

double readPressure() {
  // analog in 0-255. Pressure transducer outputs 0.5-4.5V, for 0-300 psi (0-20.68 BarG)
  double raw = (double)analogRead(PRESSURE_INPUT_PIN);
  // convert from ADC input to PSI, accounting for ADC nonlinearity
  double psi = (raw - 270.0) * 300.0 / 4095.0;

  // track moving average to reduce noise
  pressureTotal -= pressures[pressureIndex];
  pressures[pressureIndex] = psi;
  pressureTotal += pressures[pressureIndex];
  pressureIndex++;
  pressureIndex %= PRESSURE_AVG_COUNT;
  
  return pressureTotal/((double)PRESSURE_AVG_COUNT);
}

void controlPump(double currentPressure, double targetPressure) {
  pumpInput = currentPressure;
  pumpSetpoint = targetPressure;
  pumpPID.Compute();
  
  //int output = min(max((targetPressure-currentPressure), 0.0)*10, 255.0);
  analogWrite(PUMP_SPEED, pumpOutput);
} 