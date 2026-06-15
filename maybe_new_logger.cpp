#include <Arduino.h>
#include "HX711.h"

HX711 scale;

// Pins
const uint8_t dataPin = 6;
const uint8_t clockPin = 7;
const uint8_t throttlePin = 8;

// Parameters (tune these)
const float tolerance = 2.0;        // tolerance
const float tolerancePercent = 5.0;  // % tolerance
const unsigned long T_stable = 500; // ms
const unsigned long T_measure = 500; // ms
const unsigned long T_calib = 1000; // ms

// State machine
enum State { WAIT_STABLE, MEASURING, CALIBRATING };
State state = WAIT_STABLE;

// Data
float bins[101] = {0};
bool binFilled[101] = {false};

// Stability tracking
float stableValue = 0;
unsigned long stableStart = 0;
bool timerRunning = false;

// Measurement
float loadSum = 0;
uint16_t sampleCount = 0;
unsigned long measureStart = 0;
unsigned long calibStart = 0;

// Calibration
bool calibrated = false;
int minThrottle = -1;
int maxThrottle = -1;


// Calibration helper
unsigned long readCalibration() {
  return pulseIn(throttlePin, HIGH, 30000);
}

// Load Cell Calibration Function
void loadCellCalib(){
//  scale.begin(dataPin, clockPin);

  Serial.println("Load Cell Calibration Starting...");
  Serial.print("\nRemove all weight from the loadcell");
  //  flush Serial input
  while (Serial.available()) Serial.read();

  Serial.println(" and press Enter");
  while (Serial.available() == 0);

  scale.tare(20);
  int32_t offset = scale.get_offset();

  Serial.print("Place a weight on the loadcell");
  //  flush Serial input
  while (Serial.available()) Serial.read();

  Serial.print(" and enter the weight in (whole) grams and press Enter: ");
  uint32_t weight = 0;
  while (Serial.peek() != '\n')
  {
    if (Serial.available())
    {
      char ch = Serial.read();
      if (isdigit(ch))
      {
        weight *= 10;
        weight = weight + (ch - '0');
      }
    }
  }
  Serial.print("WEIGHT: ");
  Serial.println(weight);
  scale.calibrate_scale(weight, 20);
  float scaling_value = scale.get_scale();

  scale.set_offset(offset);
  scale.set_scale(scaling_value);

}



// Helpers
float readThrottle() {
  unsigned long raw = pulseIn(throttlePin, HIGH, 30000);
  if( raw <= 0){
    Serial.println("Sorry there has been an error in reading the throttle value, we are going to restart");
    state = WAIT_STABLE;
    timerRunning = false;
    return -1;
  }
  return map(raw, minThrottle, maxThrottle, 0, 100);
}

bool withinTolerance(float a, float b) {
  return abs(a - b) <= tolerance;
}

// tells if a is withing +-tolerancePercent of b, must always use b as the reference
bool withinTolerancePercent(float a, float b) {
  return abs( a/b - 1 ) <= tolerancePercent;
}

uint8_t quantize(float t) {
  if (t < 0) return 0;
  if (t > 100) return 100;
  return (uint8_t)(t + 0.5);
}

void printStatus() {
  Serial.print("Bins: ");
  for (int i = 0; i <= 100; i++) {
    if (binFilled[i]) Serial.print("x");
    else Serial.print(".");
  }
  Serial.println();
}

void reset(){
  for(int i = 0; i < 101; i++){
    bins[i] = 0;
    binFilled[i] = false;
  }
//  bins[101] = {0};
//  binFilled[101] = {false};
  state = WAIT_STABLE;
  timerRunning = false;
}




void setup() {
  Serial.begin(115200);


  scale.begin(dataPin, clockPin);
  loadCellCalib();



//  Serial.println("Ready. Sweep throttle slowly.");

  Serial.println("Throttle Calibration...\n");
  Serial.print("Put Throttle on minimum");
  while(Serial.available()) Serial.read();
  Serial.println(" and press Enter");
  while( Serial.available() == 0);


  while(!calibrated){

    unsigned long c = readCalibration();

    switch (state) {

      case WAIT_STABLE:

        if (!timerRunning) {
          stableStart = millis();
          stableValue = c;
          timerRunning = true;
        }

        if( c == 0 ){
          Serial.println("There has been an error in the measurement of the pulse, we will try again");
          state = WAIT_STABLE;
          timerRunning = false;
          break;
        }

        if(!withinTolerancePercent(c, stableValue)) {
          // reset if drifting
          timerRunning = false;
          break;
        }

        if(millis() - stableStart > T_stable) {
          // stable achieved
          stableValue = c;
          loadSum = 0;
          sampleCount = 0;
          calibStart = millis();

          state = CALIBRATING;

        }

        break;


      case CALIBRATING:

        c = readCalibration();

        if( c == 0 ){
          Serial.println("There has been an error in the measurement of the pulse, we will try again");
          state = WAIT_STABLE;
          timerRunning = false;
          break;
        }

        if(!withinTolerancePercent(c, stableValue)) {
          state = WAIT_STABLE;
          timerRunning = false;
          break;
        }

        loadSum += c;
        sampleCount++;

        if (millis() - calibStart > T_calib) {

          if (sampleCount > 0) {
            float avgValue = loadSum / sampleCount;
            
            if(minThrottle == -1){
              minThrottle = avgValue;


              Serial.print("Put Throttle to Full");
              while(Serial.available()) Serial.read();
              Serial.println(" and press Enter");
              while( Serial.available() == 0);

              state = WAIT_STABLE;
              timerRunning = false;
              break;
            }

            if(maxThrottle == -1){
              maxThrottle = avgValue;

              if( minThrottle > maxThrottle || maxThrottle - minThrottle > 1500 || maxThrottle - minThrottle < 500){
                Serial.println("There has been an issue in the calibration, we are going to start back again");
                calibrated = false;
                minThrottle = -1;
                maxThrottle = -1;

                state = WAIT_STABLE;
                timerRunning = false;
                break;
              }

              calibrated = true;
              state = WAIT_STABLE;
              timerRunning = false;
              break;
            }
          }


          state = WAIT_STABLE;
          timerRunning = false;
        }

        break;




    }
  }

  Serial.println(minThrottle);
  Serial.println(maxThrottle);

}




void loop() {


//  Serial.println("Starting Measurements");
//  while(true){}

  float t = readThrottle();

  switch (state) {

    case WAIT_STABLE:

      if(t==-1) break;

      if (!timerRunning) {
        stableStart = millis();
        stableValue = t;
        timerRunning = true;
      }

      if (!withinTolerancePercent(t, stableValue)) {
        // reset if drifting
        timerRunning = false;
        break;
      }

      if (millis() - stableStart > T_stable) {
        // stable achieved
        stableValue = t;
        loadSum = 0;
        sampleCount = 0;
        measureStart = millis();
        state = MEASURING;
      }


      if( Serial.available() ){
        int cmd = Serial.read();
        switch ( cmd ){
          case 's':
            for(int i = 0; i < 101; i++){
              Serial.print(i);
              Serial.print(", ");
              Serial.println(bins[i]);
            }

            //give the operator time to read, then reset everything
            delay(20000);
            reset();
            break;

          case 'c':
            reset();
            break;

        }

      }

      break;

    case MEASURING:

      t = readThrottle();

      if(t == -1){
        break;
      }

      if (!withinTolerancePercent(t, stableValue)) {
        // abort
        state = WAIT_STABLE;
        timerRunning = false;
        break;
      }

      if (scale.is_ready()) {
        loadSum += scale.get_units(1);
        sampleCount++;
      }

      if (millis() - measureStart > T_measure) {

        if (sampleCount > 0) {
          float avgLoad = loadSum / sampleCount;
          uint8_t bin = quantize(stableValue);

          if (!binFilled[bin]) {
            bins[bin] = avgLoad;
            binFilled[bin] = true;

            Serial.print("Stored bin ");
            Serial.print(bin);
            Serial.print(" → ");
            Serial.println(avgLoad);
          }
        }

        printStatus();

        state = WAIT_STABLE;
        timerRunning = false;
      }

      break;




  }
}