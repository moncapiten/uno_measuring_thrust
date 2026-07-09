#include <Arduino.h>
#include "HX711.h"
HX711 scale;
//#include <iostream>






struct HardwareParams{
    // Pins
    const int currentPin = A0;
    const int voltagePin = A4;
    const uint8_t scale_dataPin = 7;
    const uint8_t scale_clockPin = 6;
    const uint8_t throttlePin = 2;

};

struct SystemData {

    bool calibrated = false;
    bool measured = false;


    // system Parameters
//    const float tolerance = 2.0;        // tolerance
    const float tolerancePercent = .05;  // % tolerance
    const unsigned long T_stable = 500; // ms
    const unsigned long T_measure = 500; // ms
    const unsigned long T_calib = 1000; // ms

    // Calibration Constants
    float Rv;                               // Voltage measuring ratio
    float Rc;                               // Current measuring ratio
                                            // LoadCell is calibrated using its own library
    unsigned long minThrottle, maxThrottle; // Throttle limit 
    unsigned long rawThrottleSwap;          // Used to TEMPORARILY store the stable throttle value during calibration

    unsigned long instantaneousRawThrottle;
    float instantaneousMappedThrottle;


    unsigned long stableRawValue = 0;
    float stableMappedValue = 0;

    unsigned long stableStart = 0;
    unsigned long measureStart = 0;
    bool timerRunning = false;

    float currentSum = 0;
    float voltageSum = 0;
    float loadSum = 0;
    uint16_t loadsCount = 0;
    uint16_t sampleCount = 0;

    /* float currentBins[101] = {0};
    float voltageBins[101] = {0}; */
    float currentBins[101] = {0};
    float voltageBins[101] = {0};
    float loadBins[101] = {0};
//    bool binFilled[101] = {false};

    unsigned long long int binFilled_lowerHalf = 0;     // Using bit manipulation to track filled bins and perform checks fast
    unsigned long long int binFilled_upperHalf = 0;    // one bit per bin, 1 filled, 0 not filled


    unsigned long throttlePulseWidth = 0;
    unsigned long pulseStart = 0;


};

// adding prototypes here cause otherwise Arduino won't compile, despite pio compiling without an issue
enum class MainState;
enum class CalibSubState;
enum class ThrottleMeasureSubState;

SystemData data;
HardwareParams params;






void throttleInterrupt() {
    if (digitalRead(params.throttlePin) == HIGH) data.pulseStart = micros();
    else if (data.pulseStart > 0) data.throttlePulseWidth = micros() - data.pulseStart;
}



/* unsigned long readThrottle(){
    return pulseIn(params.throttlePin, HIGH, 30000);
} */

unsigned long readThrottle() {
    return data.throttlePulseWidth;
}











// tells if a is withing +-tolerancePercent of b, must always use b as the reference
bool withinTolerancePercent(float a, float b) {
    if (b == 0) return abs(a - b) <= 1; //abs(a) <= data.tolerancePercent;
    return abs( a/b - 1 ) <= data.tolerancePercent;
}

/* bool withinTolerance(unsigned long a, unsigned long b) {
  return abs((long)a - (long)b) <= data.tolerance;
} */



uint8_t quantize(float t) {
    if (t < 0) return 0;
    if (t > 100) return 100;
    return (uint8_t)(t + 0.5);
}




// Top level state machine, represents the entire flow of the system.
enum class MainState {
    Idle,           // default state, waiting for control inputs
    Calibration,    // calibrating sensors through sub machine
    Measurement,    // actually running a measuring loop, once again using a sub machine
    Error,
    Communication   // communicating back to the users all acquired Data
};

// Sub machine with initial clearing state, state for each sensor and final completion state
enum class CalibSubState {
    Init,
    Voltage,
    Current,
    LoadCell,
    Throttle,
    Complete
};

// Sub machine moving based on throttle stabilization, used both in calibration and measurement
enum class ThrottleMeasureSubState {
    Init,
    Stabilize,
    Calibrate,
    Measure,
};



// helpers to print states
void printState(const MainState state) {
    switch (state) {
        case MainState::Idle:
        Serial.print(F("Idle"));
        break;
        case MainState::Calibration:
        Serial.print(F("Calibration"));
        break;
        case MainState::Measurement:
        Serial.print(F("Measurement"));
        break;
        case MainState::Error:
        Serial.print(F("Error"));
        break;
        case MainState::Communication:
        Serial.print(F("Communication"));
        break;
    }
};

void printState(const CalibSubState& state) {
    switch (state) {
        case CalibSubState::Init:
        Serial.print(F("Init"));
        break;
        case CalibSubState::Voltage:
        Serial.print(F("Voltage"));
        break;
        case CalibSubState::Current:
        Serial.print(F("Current"));
        break;
        case CalibSubState::LoadCell:
        Serial.print(F("LoadCell"));
        break;
        case CalibSubState::Throttle:
        Serial.print(F("Throttle"));
        break;
        case CalibSubState::Complete:
        Serial.print(F("Complete"));
        break;
    }
};

void printState(const ThrottleMeasureSubState& state) {
    switch (state) {
        case ThrottleMeasureSubState::Init:
        Serial.print(F("Init"));
        break;
        case ThrottleMeasureSubState::Stabilize:
        Serial.print(F("Stabilize"));
        break;
        case ThrottleMeasureSubState::Calibrate:
        Serial.print(F("Calibrate"));
        break;
        case ThrottleMeasureSubState::Measure:
        Serial.print(F("Measure"));
        break;
    }
};




























void printBits64(unsigned long long num) {
  // 64 bits + 1 null terminator = 65 bytes
  char buffer[65]; 
  buffer[64] = '\0'; // Null-terminate the string

  // Fill the buffer from right to left (LSB to MSB)
  for (int i = 63; i >= 0; i--) {
    buffer[i] = (num & 1) ? '1' : '0';
    num >>= 1; // Shift right by 1 bit
  }

  // Send the entire block to the hardware serial buffer at once
  Serial.write(buffer, 64);
}

void printBits36(unsigned long long num) {
  // 36 bits + 1 null terminator = 37 bytes
  char buffer[37]; 
  buffer[36] = '\0'; // Null-terminate the string

  // Fill the buffer from right to left (LSB to MSB)
  for (int i = 35; i >= 0; i--) {
    buffer[i] = (num & 1) ? '1' : '0';
    num >>= 1; // Shift right by 1 bit
  }

  // Send the entire block to the hardware serial buffer at once
  Serial.write(buffer, 36);
}

void printCalibration(const SystemData& data_to_print){
    Serial.println(F("\n➔ Calibration Data:"));
    Serial.print(F("\tVoltage Ratio (Rv): "));
    Serial.println(data_to_print.Rv);
    Serial.print(F("\tCurrent Ratio (Rc): "));
    Serial.println(data_to_print.Rc);
    Serial.print(F("\tMin Throttle: "));
    Serial.println(data_to_print.minThrottle);
    Serial.print(F("\tMax Throttle: "));
    Serial.println(data_to_print.maxThrottle);
}

void send_all_data(const SystemData& data_to_send){
    Serial.println(F("\n➔ Sending Data..."));

    printCalibration(data_to_send);

    Serial.println(F("\nTh[%],\tL[g],\tV[V],\tI[A]"));

    for(int i = 0; i <= 100; i++){
//        if( (data.binFilled_lowerHalf & (1ULL << i)) || (data.binFilled_upperHalf & (1ULL << (i-64))) ){
        Serial.print(i);
        Serial.print(F(",\t"));
        Serial.print(data.loadBins[i]);
        Serial.print(F(",\t"));
        Serial.print(data.voltageBins[i] * data.Rv );
        Serial.print(F(",\t"));
        Serial.println(data.currentBins[i] * data.Rc);
//        }
    }
}






struct MachineController {
    MainState currentMainState = MainState::Idle;
    MainState previousMainState = MainState::Error; // In order to print the welcome message as we start
    CalibSubState currentCalibState = CalibSubState::Init;
    ThrottleMeasureSubState currentThrottleMeasureState = ThrottleMeasureSubState::Init;
    CalibSubState previousCalibState = CalibSubState::Complete;
    ThrottleMeasureSubState previousThrottleMeasureState = ThrottleMeasureSubState::Measure;

    bool stateChanged(){
        if (currentMainState != previousMainState){
            previousMainState = currentMainState;
            return true;
        }
        return false;
    }
    bool calibStateChanged(){
        if (currentCalibState != previousCalibState){
            previousCalibState = currentCalibState;
            return true;
        }
        return false;
    }
    bool throttleMeasureStateChanged(){
        if (currentThrottleMeasureState != previousThrottleMeasureState){
            previousThrottleMeasureState = currentThrottleMeasureState;
            return true;
        }
        return false;
    }
    // General Purpose variables;
//    bool calibrated = false;

    void printAllStates(){
        Serial.println();
        printState(currentMainState);
        Serial.print(" | ");
        printState(currentCalibState);
        Serial.print(" | ");
        printState(currentThrottleMeasureState);
        Serial.println();
    }

    // Run this inside your main embedded loop
    void update() {
        data.instantaneousRawThrottle = readThrottle();
        if (data.calibrated) data.instantaneousMappedThrottle = map(data.instantaneousRawThrottle, data.minThrottle, data.maxThrottle, 0UL, 100UL);
//        Serial.print(data.instantaneousRawThrottle);
/*         if( data.calibrated){
            Serial.print(" ⇒ ");
            Serial.print(data.instantaneousMappedThrottle);
            Serial.print("% **** ");
            Serial.print(data.minThrottle);
            Serial.print(" <-> ");
            Serial.println(data.maxThrottle);
        } */

/*        if( stateChanged() || calibStateChanged() || throttleMeasureStateChanged() )  printAllStates();**************************************************************************************************************************************************************************************************************************/


        switch (currentMainState) {
            case MainState::Idle: {
                // Run Calibration if not yet calibrated
                // Otherwise wait for commands

                if( data.calibrated == false){
                    Serial.println(F("➔ Device is not Calibrated"));
                    currentCalibState = CalibSubState::Init;
                    currentMainState = MainState::Calibration;
                    break;
                }

                if (stateChanged()) {
                    Serial.println(F("\n➔ Current State: Idle"));
                    
                    Serial.println(F("➔ Press 'c' to start a calibration routine"));
                    Serial.println(F("➔ Press 'm' to start a measurement routine"));
                    Serial.println(F("➔ Press 's' to start a data comms routine"));
                    while(Serial.available()) Serial.read();
                    Serial.print(F("➔ "));
                }
                /* ADD ABILITY TO READ THE SERIAL INPUT*/
                while(Serial.available() == 0);
                char input = Serial.read();
                Serial.print(F("➔ INPUT: "));
                Serial.println(input);
                switch(input){
                    case 'c':
                        data.calibrated = false; // Reset calibration flag to ensure calibration routine runs
                        currentCalibState = CalibSubState::Init;
                        currentMainState = MainState::Calibration;
                        break;
                    case 'm':
                        if (data.calibrated == false){
                            Serial.println(F("➔ Device is not Calibrated, please calibrate first"));
                            break;
                        } else {
                            currentMainState = MainState::Measurement;
                            currentThrottleMeasureState = ThrottleMeasureSubState::Init;
                            break;
                        }
                    case 's':
                        if (data.measured == false){
                            Serial.println(F("➔ No data measured yet, please run a measurement first"));
                            break;
                        } else {
                            currentMainState = MainState::Communication;
                            break;
                        }
                    default:
                        Serial.println(F("➔ Invalid input, please try again\n➔ "));

                    }

                    break; // YOU GORFOT THIS YOU FUCKING IDIOT
                }
                    

            case MainState::Calibration:
                if(stateChanged()){
                    Serial.println(F("\n➔ Current State: Calibration"));
                    currentCalibState = CalibSubState::Init; // Reset sub-state
                }
                // Call the dedicated machine handler
                runCalibrationSubMachine();
                break;

            case MainState::Measurement:
                if(stateChanged()){
                    Serial.println(F("\n➔ Current State: Measurement"));
                    currentThrottleMeasureState = ThrottleMeasureSubState::Init; // Reset sub-state
                }

//                currentThrottleMeasureState = ThrottleMeasureSubState::Init;
                runThrottleSubmachine();

                break;

            case MainState::Communication:
                if(stateChanged()) Serial.println(F("\n➔ Current State: Communication"));
                // Handle communication with the user, sending acquired data, etc.

                send_all_data(data);

                currentMainState = MainState::Idle; // After communication, return to idle or end as needed
                break;

            case MainState::Error:
                // Handle errors
                // doesnt really do shit as of now
                break;
        }
    }

private:
    void runCalibrationSubMachine() {
        switch (currentCalibState) {
            case CalibSubState::Init:
                Serial.println(F("\n➔ Starting Calibration..."));
                
                data.Rc = 0;
                data.Rv = 0;
                data.minThrottle = -1;
                data.maxThrottle = -1;


                currentCalibState = CalibSubState::Voltage;
                break;

            case CalibSubState::Voltage:
                Serial.println(F("\n➔ Calibrating Voltage Sensor..."));
                
                while(Serial.available()) Serial.read();
                Serial.print(F("Please input the Voltage measuring ratio Rv ( actual voltage = measured Voltage * Rv):\t"));
                while(Serial.available() == 0);
                data.Rv = Serial.parseFloat();
//                if( Serial.available() ) data.Rv = Serial.parseFloat();

                Serial.println(data.Rv);
                currentCalibState = CalibSubState::Current;
                break;

            case CalibSubState::Current:
                Serial.println(F("\n➔ Calibrating Current Sensor..."));

                while(Serial.available()) Serial.read();
                Serial.print(F("Please input the Current measuring ratio Rc ( actual current = measured current * Rc):\t"));
                while(Serial.available() == 0);
                data.Rc = Serial.parseFloat();

                Serial.println(data.Rc);
                currentCalibState = CalibSubState::LoadCell;
                break;

            case CalibSubState::LoadCell: {
                Serial.println(F("\n➔ Calibrating Load Cell..."));

                Serial.print(F("➔ Remove all weight from the loadcell"));
                while (Serial.available()) Serial.read();
                Serial.print(F(" and press Enter\t"));
                while (Serial.available() == 0);
                Serial.println(F("✓"));

                scale.tare(20);
                int32_t offset = scale.get_offset();

                Serial.print(F("➔ Place a weight on the loadcell"));
                while (Serial.available()) Serial.read();
                Serial.print(F(" and enter the weight in (whole) grams and press Enter:\t"));

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

//                Serial.print(F("WEIGHT: "));
                Serial.println(weight);
                scale.calibrate_scale(weight, 20);
                float scaling_value = scale.get_scale();

                scale.set_offset(offset);
                scale.set_scale(scaling_value);

                currentCalibState = CalibSubState::Throttle;
                break;
            }

            case CalibSubState::Throttle:
                Serial.println(F("\n➔ Calibrating Throttle..."));

                while(Serial.available()) Serial.read();
                Serial.print(F("➔ Set throttle to minimum"));
                delay(1000);
                Serial.print(F(" and press Enter - "));
                currentThrottleMeasureState = ThrottleMeasureSubState::Stabilize;
                while(Serial.available() == 0){
                    data.instantaneousRawThrottle = readThrottle();
                    runThrottleSubmachine();
                };

//                currentThrottleMeasureState = ThrottleMeasureSubState::Stabilize;
//                runThrottleSubmachine();
                data.minThrottle = data.rawThrottleSwap;
                Serial.println(data.minThrottle);

                while(Serial.available()) Serial.read();
                data.rawThrottleSwap = 0;

                while(Serial.available()) Serial.read();
                Serial.print(F("➔ Set throttle to maximum"));
                delay(1000);
                Serial.print(F(" and press Enter - "));
                currentThrottleMeasureState = ThrottleMeasureSubState::Stabilize;
                while(Serial.available() == 0){
                    data.instantaneousRawThrottle = readThrottle();
                    runThrottleSubmachine();
                }

/*                 currentThrottleMeasureState = ThrottleMeasureSubState::Stabilize;
                runThrottleSubmachine();
 */
                data.maxThrottle = data.rawThrottleSwap;
                Serial.println(data.maxThrottle);
                data.rawThrottleSwap = 0;

                currentCalibState = CalibSubState::Complete;
                break;

            case CalibSubState::Complete:
/*                 if( !data.calibrated ){
                    Serial.println(data.Rv);//******************************************************************************************************************************************************************************************************************
                    Serial.println(data.Rc);//******************************************************************************************************************************************************************************************************************
                    Serial.println(data.minThrottle);//*********************************************************************************************************************************************************************************************************
                    Serial.println(data.maxThrottle);/*********************************************************************************************************************************************************************************************************
                } */

                printCalibration(data);


                Serial.println(F("\n➔ Calibration Complete!"));
                data.calibrated = true; // Set the flag to indicate calibration is done
                currentMainState = MainState::Idle; // Return to idle or move to measurement
                currentThrottleMeasureState = ThrottleMeasureSubState::Init; // Reset throttle sub-state for future use
                break;
        }
    }


    void runThrottleSubmachine(){
//    void runThrottleSubmachine(const bool calibrationMode) {
        
//        if (calibrationMode) unsigned long instantaneousThrottle = readThrottle(AsRaw{});
//        else float instantaneousThrottle = readThrottle(AsMapped{});
        
        switch (currentThrottleMeasureState) {

            case ThrottleMeasureSubState::Init:
                data.stableRawValue = 0;
                data.instantaneousMappedThrottle = 0;

                data.stableStart = 0;
                data.measureStart = 0;
                data.timerRunning = false;

                data.currentSum = 0;
                data.voltageSum = 0;
                data.loadSum = 0;
                data.sampleCount = 0;
                data.loadsCount = 0;

                memset(data.currentBins, 0, sizeof(data.currentBins));
                memset(data.voltageBins, 0, sizeof(data.voltageBins));
                memset(data.loadBins, 0, sizeof(data.loadBins));

                data.binFilled_lowerHalf = 0;
                data.binFilled_upperHalf = 0;


                Serial.print(F("\n➔ Starting Throttle Measurement Routine..."));
                Serial.print(data.minThrottle);
                Serial.print(F(" _-_-_-_-_ "));
                Serial.println(data.maxThrottle);

                // FIND A WAY TO CLEAR THE BINS WITHOUT HAVING TO LOOP THROUGH THEM

                /* float currentBins[101] = {0};
                float voltageBins[101] = {0}; */
/*                 int currentBins[101] = {0};
                int voltageBins[101] = {0};
                float loadBins[101] = {0}; */
            //    bool binFilled[101] = {false};

/*                 unsigned long long int binFilled_lowerHalf = 0;     // Using bit manipulation to track filled bins and perform checks fast
                unsigned long long int binFilled_upperHalf = 0;    // one bit per bin, 1 filled, 0 not filled */
            

                currentThrottleMeasureState = ThrottleMeasureSubState::Stabilize;
                break;


            case ThrottleMeasureSubState::Stabilize:
                if(data.calibrated == false){
//                if( calibrationMode == true){
                    
                    if (!data.timerRunning) {
                        data.stableStart = millis();
                        data.stableRawValue = data.instantaneousRawThrottle;
                        data.timerRunning = true;
                    }

                    if (!withinTolerancePercent(data.instantaneousRawThrottle, data.stableRawValue)) {
                        // reset if drifting
                        data.timerRunning = false;
                        break;
                    }

                    if (millis() - data.stableStart > data.T_stable) {
                        // stable achieved
                        data.stableRawValue = data.instantaneousRawThrottle;
                        data.loadSum = 0;
                        data.sampleCount = 0;
                        data.loadsCount = 0;
                        data.measureStart = millis();
                        currentThrottleMeasureState = ThrottleMeasureSubState::Calibrate;
                    }
                    break;

                } else {

                    if (!data.timerRunning) {
                        data.stableStart = millis();
                        data.stableMappedValue = data.instantaneousMappedThrottle;
                        data.timerRunning = true;
                    }

                    if (!withinTolerancePercent(data.instantaneousMappedThrottle, data.stableMappedValue)) {
                        // reset if drifting
                        data.timerRunning = false;
                        break;
                    }

                    if (millis() - data.stableStart > data.T_stable) {
                        // stable achieved
                        data.stableMappedValue = data.instantaneousMappedThrottle;
                        data.currentSum = 0;
                        data.voltageSum = 0;
                        data.loadSum = 0;
                        data.sampleCount = 0;
                        data.loadsCount = 0;
                        data.measureStart = millis();
                        currentThrottleMeasureState = ThrottleMeasureSubState::Measure;
                    }


                    
                    break;

                }



            case ThrottleMeasureSubState::Calibrate:
//                if( data.calibrated == true){
                data.rawThrottleSwap = data.stableRawValue; //value has already been stabilized, so let's save it
                return;
                break;



            case ThrottleMeasureSubState::Measure:

                if (!withinTolerancePercent(data.instantaneousMappedThrottle, data.stableMappedValue)) {
                    // abort
                    currentThrottleMeasureState = ThrottleMeasureSubState::Stabilize;
                    data.timerRunning = false;
                    break;
                }

                data.currentSum += analogRead(params.currentPin);
                data.voltageSum += analogRead(params.voltagePin);
                data.sampleCount++;

                if (scale.is_ready()) {
/*                     data.currentSum += analogRead(params.currentPin);
                    data.voltageSum += analogRead(params.voltagePin); */
                    data.loadSum += scale.get_units(1);
                    data.loadsCount++;
                }

                if (millis() - data.measureStart > data.T_measure) {

                    if (data.sampleCount > 0) {
                        float avgCurrent = data.currentSum / data.sampleCount * data.Rc;
                        float avgVoltage = data.voltageSum / data.sampleCount * data.Rv;
/*                         int avgCurrent = (int)(data.currentSum / data.sampleCount);
                        int avgVoltage = (int)(data.voltageSum / data.sampleCount); */
                        float avgLoad = data.loadSum / data.loadsCount;                        

                        uint8_t bin = quantize(data.stableMappedValue);

                        // Store avgLoad in the corresponding bin
                        data.currentBins[bin] = avgCurrent;
                        data.voltageBins[bin] = avgVoltage;
                        data.loadBins[bin] = avgLoad; // Assuming bins is defined somewhere
//                        data.binFilled[bin] = true; // Mark this bin as filled

                        if(bin < 64){
                            data.binFilled_lowerHalf |= (1ULL << bin);
                        } else {
                            data.binFilled_upperHalf |= (1ULL << (bin - 64));
                        }


                        //TODO: make sure the measurement runs each turn and that
                        // there is an way to exit, either when all bins are filled
                        // or through a user command

/*                         Serial.print(F("Stored bin "));
                        Serial.print(bin);
                        Serial.print(F(" → "));
                        Serial.println(avgLoad);*/
                        Serial.print(F("➔ "));
                        printBits36(data.binFilled_upperHalf);
                        printBits64(data.binFilled_lowerHalf);
                        Serial.print(F(" - "));
                        Serial.print(bin);
                        Serial.print(F("( "));
                        Serial.print(data.instantaneousRawThrottle);
                        Serial.print(F(") - "));
                        Serial.print(-avgLoad);
                        Serial.print(F("g ("));
                        Serial.print(data.loadsCount);
                        Serial.print(F(" samples) - "));
                        Serial.print(avgVoltage * 0.019528);
//                        Serial.print(map(avgVoltage, 0, 1023, 0, 15));
                        Serial.print(F("V - "));
                        Serial.print(-( (avgCurrent - 507.4) *0.073969));
//                        Serial.print(avgCurrent);
                        Serial.print(F(" ("));
                        Serial.print(data.sampleCount);
                        Serial.println(F(" samples)"));


                    }

                    // printStatus(); // Optionally print status

                    if( (data.binFilled_lowerHalf ^ 0xFFFFFFFFFFFFFFFFULL) == 0 && (data.binFilled_upperHalf ^ 0x1FFFFFFFFF) == 0 ){
                        Serial.println(F("All bins filled, ending measurement routine."));
                        data.measured = true;
                        currentMainState = MainState::Idle;
                        break;
                    }

                    if(Serial.available() > 0){
                        char input = Serial.read();
                        if(input == 'q'){
                            Serial.println(F("➔ Exiting Measurement Routine..."));
                            currentMainState = MainState::Idle;
                            data.timerRunning = false;
                            data.measured = true;
                            break;
                        }
                    }

                    currentThrottleMeasureState = ThrottleMeasureSubState::Stabilize;
                    data.timerRunning = false;
                }
                break;
        }
    }

};





























MachineController machine;

void setup(){
    Serial.begin(115200);
    scale.begin(params.scale_dataPin, params.scale_clockPin);


    attachInterrupt(digitalPinToInterrupt(params.throttlePin), throttleInterrupt, CHANGE); // Attaching Interrupt for throttle measurement to be fast nad not having to throttle( hehe) the measurements

    pinMode(params.currentPin, INPUT);
    pinMode(params.voltagePin, INPUT);
    

//    machine.update(); // Call update once to start the state machine

}


void loop() {
    machine.update(); // Continuously call update to run the state machine
}


