#include <avr/wdt.h>
#include "ApplicationFunctionSet_xxx0.h"
#include <SoftTimers.h>
#include <arduino.h>
#include <TimerFreeTone.h>
#include <FastLED.h>
#include <stdlib.h>


const int pins[] = {A8, A9, A10, A11};
const int dir[] = {45, 45, 135, 135};
const int leftMics[] = {A8, A11};
//float calib[] = {1.0f,1.43f,1.0f,2.22f};
float calib[] = {1.2f,1,1,1.3f};
const int pin_size = 4;
const int threshold = 200;

bool nd = false;
int cnt[pin_size][2];

enum RobiStates{
    Idle,
    Alerted,
    Hunting,
    Init
};

enum DrivingStates{
    Random,
    Forward,
    Turning,
    DoNothing
};
 
// SoftTimers: 1000 = 1sec

const int pirPin = 2;
const int alertedTimeThreshold = 15000; //15 sec
const int PIN_RBGLED = 4;
const int NUM_LEDS = 1;
const int buzzerPin = 53;

// könnte man als enum machen, habe ich nur nicht direkt hinbekommen weil das in der Funktion DeviceDriverSet_xxx0.cpp verwendet wird
const int TURN_LEFT = 0; 
const int TURN_RIGHT = 1;

SoftTimer stateTimer;
SoftTimer driveTimer;
//SoftTimer turnTimer; // maybe not needed -> depends on states propably

RobiStates currentState;
DrivingStates currentDrivingState;
int timerCounter = 0;
int timeInAlerted = 0;
int timerThreshold = 5;
int timerDriveCounter = 0;
int timerDriveThreshold = 5000;
int timerTurnThreshold = 2000;

int testTimeInIdle = 0;
int testCounterHunting = 0;

int micCounter = 1;

//long buzzerFrequence = 1200;
//long buzzerDuration = 100;
int melody[] = { 150, 200, 250, 0, 150, 200, 250};
int duration[] = { 500, 500, 500, 500, 500, 500, 500};
int melodyFound[] = {250, 250, 250, 0, 250, 250, 250};
int durationFound[] = { 500, 500, 500, 500, 500, 500, 500};

//initilize car speed for initial mode
uint8_t speed = 100;
// initialize meters to drive formard in cm --> not used in current version 
int forwardDistance = 40; 
int turnAngle = 90; // 90 degree

volatile bool enabledDrive = false;
volatile bool enabledTurn = false;
volatile bool enabledHunting = false;
volatile bool enabledMicArray = true;

CRGB leds[NUM_LEDS]; 

typedef void (*voidFunction) (void); 
voidFunction callbackStateFunction;
voidFunction callbackDriveStateFunction;


void setup(){
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    Application_FunctionSet.ApplicationFunctionSet_Init();

    //stateTimer.setTimeOutTime(timerThreshold);
    //stateTimer.reset();

    FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
    FastLED.setBrightness(200);
    initRobi();

    for(int i = 0; i < pin_size; i++){
        pinMode(pins[i], INPUT);
    }

}

void loop(){
    if (stateTimer.hasTimedOut()){
      digitalWrite(LED_BUILTIN, HIGH);
      enabledMicArray = false;
      callbackStateFunction();
      stateTimer.reset();
    }
    if (driveTimer.hasTimedOut()){
        
      if (currentState == Idle){
        leds[0] = CRGB::Green;
        FastLED.show();
      }
      if (currentState == Alerted){
        leds[0] = CRGB::Orange;
        FastLED.show();
      }
        
      enabledDrive = false;
      int driverTimeOutTime = stateTimer.getTimeOutTime() + 1000; //to guarantee that drivertimer does not timeout earlier than state
      driveTimer.setTimeOutTime(driverTimeOutTime);
      driveTimer.reset();
      stateTimer.reset();
      // delay(100);
      //enabledMicArray = true;
    }

    if (enabledDrive == false && currentState != Init && stateTimer.getElapsedTime()>1000){
      enabledMicArray = true;
    }
    
    if (enabledDrive == true){
        callbackDriveStateFunction();
    }else{
        
        if (currentDrivingState == Turning){
            setDrivingState(Forward);
        }else if(currentDrivingState == Forward){
            //Hunting ends, enemy caught
            setDrivingState(DoNothing);
            int arrSize = sizeof(melodyFound)/sizeof(melodyFound[0]);
            for (int thisNote = 0; thisNote < arrSize; thisNote++) { // Loop through the notes in the array.
                TimerFreeTone(buzzerPin, melodyFound[thisNote], durationFound[thisNote]); // Play thisNote for duration.
            }
            setRobiState(Idle);
        }
        
        else{
            setDrivingState(DoNothing);
        }
        
      Application_FunctionSet.ApplicationFunctionSet_StopRobi();   
    }

    if (enabledMicArray){
        micDetect();
    }
    
}

void initRobi(){
    setRobiState(Init);
    setDrivingState(DoNothing);
    pinMode(pirPin, INPUT);
    Serial.begin(9600);
    
}

void micDetect(){
  int raw_values[pin_size];
  int values[pin_size];
  for(int i = 0; i < pin_size; i++){
      raw_values[i] = (float)(analogRead(pins[i])-512)*calib[i];
  }
  for(int i = 0; i < pin_size; i++){
    values[i] = abs((values[i] + raw_values[i]) / 2);
  }
  
  bool new_noise = false;
  for(int i = 0; (i < pin_size); i++){
    if(values[i] >= threshold){
      new_noise = true;
    }
  }

  
  if(new_noise && nd){
    //Continued noise
    Serial.println("Noise continued");
    int max_pin = getMaxPin(values);
    increaseMaxPinCount(max_pin);
    printValues(values);
  }
  else if(nd && !new_noise){
    //noise ended
    Serial.println("Noise ended");
    nd = false; 
    int sortedPins[pin_size];
    for(int i = 0; i < pin_size; i++){
      sortedPins[i] = i;
    }
    //sort
    for(int i = 0; i < pin_size; i++){
      for(int j = i; j < pin_size; j++){
        if(cnt[i][1] < cnt[j][1]){
          int temp = sortedPins[i];
          sortedPins[i] = sortedPins[j];
          sortedPins[j] = temp;
        }
      }
    }

    for(int i = 0; i < pin_size; i++){
      Serial.print(sortedPins[i]);
    }
    Serial.println("");

    //Direction by pin with most counts
    int first_pin = sortedPins[0];
    int noise_direction = dir[first_pin];

    //Addition by pin with second most counts
    int second_pin = sortedPins[1];
    if(((first_pin - second_pin) % 2) == 1){
      // Pins are next to each other
      if(cnt[second_pin][1] != 0){
        // second Pin received signals
        float quot = (1.0f * cnt[second_pin][1]) / cnt[first_pin][1];
        if(quot > 0.25){
          // second pin received significant amount of signal
          float piece = 360.0f / (2.0f * pin_size);
          int added_angle = piece * quot;
          
          if(second_pin == ((first_pin - 1) % pin_size)){
            added_angle = -added_angle;  
          }

          noise_direction = noise_direction + added_angle;
        }
      }
    }
    for (int i=0; i<2; i++){
        if(leftMics[i] == first_pin){
            noise_direction = -noise_direction;
            break;
        }
    }
    noiseDetected(noise_direction);
    Serial.println(noise_direction);
  }
  else if(!nd && new_noise){
    //Noise Started
    Serial.println("Noise started");
    printValues(values);
    nd = true;
    for(int i = 0; i < pin_size; i++){
      cnt[i][0] = pins[i];
      cnt[i][1] = 0;
    }
    int max_pin = getMaxPin(values);
    increaseMaxPinCount(max_pin);
  }
  else{
    //nothing new
  }
}

void printValues(int values[]){
  for(int i = 0; i < pin_size; i++){
    Serial.print(values[i]);
    Serial.print(" ");
  }
  Serial.println("");
}

void increaseMaxPinCount(int maxPin){
  for(int j = 0; j < pin_size; j++){
    if(maxPin == cnt[j][0]){
      cnt[j][1]++;
    }
  }
}

int getMaxPin(int values[]){
  int max_out = 0;
  int max_pin = 0;
  for(int i = 0; i < pin_size; i++){
    if(values[i] > max_out){
      max_out = values[i];
      max_pin = pins[i];
    }
  }
  return max_pin;
}

void noiseDetected(float direction){
    //micCounter = micCounter - 1;
    if(currentState != Init){
        Serial.println("noise detected executed");
        Serial.println(enabledDrive);
        killMovement();
        setForwardDistance(40);
        setTurnAngle((int) direction);
        if(currentState == Idle){
            setRobiState(Alerted);
        }
        else if(currentState == Alerted){
            setRobiState(Hunting);
        }
    }
    
    
}

void killMovement(){
    enabledDrive = false;
    setDrivingState(DoNothing);
}

void setRobiState(RobiStates newState){
    if (currentState != newState){
        //micCounter = 1;
        switch (newState)
            {
            case Init:
                leds[0] = CRGB::Blue;
                FastLED.show();
                timerThreshold = 15000; //15sec
                stateTimer.setTimeOutTime(timerThreshold);
                stateTimer.reset();
                //enabledMicArray = true;
                callbackStateFunction = []() -> void {
                    attachInterrupt(digitalPinToInterrupt(pirPin), pirISR, CHANGE);
                    
                    setRobiState(Idle);
                };
                break;
            case Idle:
                digitalWrite(LED_BUILTIN, HIGH);
                leds[0] = CRGB::Green;
                FastLED.show();
                timerThreshold = 10000; //alle 10sec bewegen
                stateTimer.setTimeOutTime(timerThreshold);
                stateTimer.reset();
                enabledMicArray = true;

                //testTimeInIdle = 0;

                callbackStateFunction = []() -> void{
                    //move around random
                    
                    setDrivingState(Random);
                    
                    /*if (testTimeInIdle>1){ //2
                        setRobiState(Alerted);
                    }else{
                        testTimeInIdle = testTimeInIdle + 1;
                        Serial.println("Exec Random");
                        setDrivingState(Random);
                    }*/
                    
                    
                };
                
                break;
            case Alerted:
                leds[0] = CRGB::Orange;
                FastLED.show();
                timerThreshold = 5000; //alle 5 sec bewegen
                stateTimer.setTimeOutTime(timerThreshold);
                stateTimer.reset();
                timeInAlerted = 0;
                //enabledMicArray = true;
                //int arrSize = sizeof(melody)/sizeof(melody[0]);
                //for (int thisNote = 0; thisNote < arrSize; thisNote++) { // Loop through the notes in the array.
                //    TimerFreeTone(buzzerPin, melody[thisNote], duration[thisNote]); // Play thisNote for duration.
                //}
                //Test counter here
                //testCounterHunting = 0;

                callbackStateFunction = []() -> void{
                    // move around faster
                    setDrivingState(Random);
                    timeInAlerted = timeInAlerted + timerThreshold;
                    if (timeInAlerted > alertedTimeThreshold){
                        setRobiState(Idle);
                    }
                    /*if (testCounterHunting > 0){ //1
                        setRobiState(Hunting);
                    }
                    testCounterHunting = testCounterHunting + 1;*/
                };
                
                break;
            case Hunting:
                // catch & follow the sound
                leds[0] = CRGB::Red;
                FastLED.show();

                //enabledMicArray = true;
                //int arrSize = sizeof(melody)/sizeof(melody[0]);
                //for (int thisNote = 0; thisNote < arrSize; thisNote++) { // Loop through the notes in the array.
                //    TimerFreeTone(buzzerPin, melody[thisNote], duration[thisNote]); // Play thisNote for duration.
                //}
                //stateTimer.setTimeOutTime(100000);
                //stateTimer.reset();
                //Get turn angle and distance from micro
                //Set variables with setter methods (setTurnAngle, setForwardDistance)
                //setTurnAngle(90);
                //setForwardDistance(60);
                setDrivingState(Turning);
                //set variables, direction of sound, setDrivingState(Turning), setDrivingState(Forward)
                callbackStateFunction = []() -> void{
                
                };
                
                break;
            default:
                break;
            }
        currentState = newState;
    }
}

void setDrivingState(DrivingStates newState){
       
        switch (newState)
            {
            case Random:
                leds[0] = CRGB::White;
                    FastLED.show();
                Serial.println("Setting Random state");
                timerDriveThreshold = 4000; //4sec fahren
                driveTimer.setTimeOutTime(timerDriveThreshold);
                driveTimer.reset();
                enabledDrive = true;
                callbackDriveStateFunction = []() -> void {
                    //enabledMicArray = false;
                    Application_FunctionSet.ApplicationFunctionSet_Obstacle();
                };
                
                break;
            case Forward:
                // MH: define meters (global variable) to drive forward
                // MH: define speed (global variable) to drive with 
                // MH:call function to calculate timerDriveTreshold (similar to getTurnTimerTreshold())
                //int forwardDistance = 53;
                timerDriveThreshold = getForwardTimeThreshold(getForwardDistance());
                Serial.println("Forward Mode");
                Serial.println(timerDriveThreshold);
                driveTimer.setTimeOutTime(timerDriveThreshold);
                driveTimer.reset();
                enabledDrive = true;
                enabledMicArray = false;
                callbackDriveStateFunction = []() -> void {
                    //enabledMicArray = false;
                    Application_FunctionSet.ApplicationFunctionSet_DriveRobi(speed);
                };
                break;
      
            case Turning:
                // define turnAngle and get accordingly timerTurnTreshold
                //turnAngle = 90; // future: will be defined by sensor
                timerDriveThreshold = getTurnTimerThreshold(getTurnAngle());
                Serial.print("Angle Threshold: ");
                Serial.println(timerDriveThreshold);
                driveTimer.setTimeOutTime(timerDriveThreshold);
                driveTimer.reset();
                enabledDrive = true;
                enabledMicArray = false;
                callbackDriveStateFunction = []() -> void{
                    //enabledMicArray = false;
                    int direction = TURN_RIGHT;
                    Serial.print("Angle direction: ");
                    Serial.println(getTurnAngle());
                    if(getTurnAngle()<0){
                        direction = TURN_LEFT;
                    }
                   Application_FunctionSet.ApplicationFunctionSet_TurnRobi(direction, speed); 
                };
                
                break;
            case DoNothing:
                //
                enabledDrive = false;            
                //leds[0] = CRGB::Green;
                //    FastLED.show();
                Application_FunctionSet.ApplicationFunctionSet_StopRobi();
                //enabledMicArray = true;
                //callbackDriveStateFunction = []() -> void{
                //    Application_FunctionSet.ApplicationFunctionSet_StopRobi();
                //};
                
                break;
            default:
                break;
            }
        currentDrivingState = newState;
    
}

void pirISR(){
    int pirStat = digitalRead(pirPin);
    if ((currentState == Idle) && (pirStat == HIGH)){
        setRobiState(Alerted);
    }else if ((currentState == Alerted) && (pirStat == HIGH)){
        timeInAlerted = 0;
    }
    
}

// for now only calibrated for speed = 100 -> need to be defined also for other speeds used in our state machine
// If we have only defined speeds w can also use states instead of speed ranges maybe
// for left turn calibration works good, but not 100% exact for right turn --> maybe differentiation here also in the future
int getTurnTimerThreshold(int turnAngle){
  int threshold = 0;
  if (speed > 75 && speed < 125){
    int offset = 100; // offset to get car into motion for speed 100 with turn of ~10 degree
    static float k = 8;
    static float threshold_float = float(turnAngle - 10) * k;
    return threshold = offset + int(threshold_float);
  }
  // weitere Speed bereiche definieren --> k & offset ...
  else{
    return threshold;
  }
}

// distance in cm
int getForwardTimeThreshold(int distance){
    int threshold = 0;
    if (speed > 75 && speed < 125){
        if (distance<=20){
            static int k = 50;
            return threshold = distance * k;
        }else{
            float adjustedDistance = distance - 20;
            float calibrate = adjustedDistance / 40;
            float readjustedDistance = calibrate + 1;
            return threshold = readjustedDistance * 1000;
        }
    }else{
        return threshold;
    }
}

void setTurnAngle(int angle){
    turnAngle = angle;
}

//distance in cm
void setForwardDistance(int distance){
    forwardDistance = distance;
}

int getTurnAngle(){
    return turnAngle;
}

int getForwardDistance(){
    return forwardDistance;
}
