#include <avr/wdt.h>
#include "ApplicationFunctionSet_xxx0.h"
#include <SoftTimers.h>
#include <arduino.h>
#include <TimerFreeTone.h>
#include <FastLED.h>
#include <stdlib.h>

// For mic array ##############
const int pins[] = {A8, A9, A10, A11};
const int dir[] = {45, 45, 135, 135};
const int leftMics[] = {A8, A11};
//float calib[] = {1.0f,1.43f,1.0f,2.22f};
float calib[] = {1.2f,1,1,1.3f};
const int pin_size = 4;
const int threshold = 200;

bool nd = false;
int cnt[pin_size][2];

// State machine #############
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

RobiStates currentState;
DrivingStates currentDrivingState;


// Defining constants
const int pirPin = 2;
const int alertedTimeThreshold = 15000; //15 sec
const int PIN_RBGLED = 4;
const int NUM_LEDS = 1;
const int buzzerPin = 53;
const int TURN_LEFT = 0; 
const int TURN_RIGHT = 1;

// SoftTimers: 1000 = 1sec
SoftTimer stateTimer;
SoftTimer driveTimer;

int timeInAlerted = 0;
int timerThreshold = 5;
int timerDriveThreshold = 5000;

// Melody and duration of tones if hunting object was found
int melodyFound[] = {250, 250, 250, 0, 250, 250, 250};
int durationFound[] = { 500, 500, 500, 500, 500, 500, 500};

//initilize car speed for initial mode
uint8_t speed = 100;
// initialize meters to drive forward in cm 
int forwardDistance = 40; 
int turnAngle = 90; // 90 degree

volatile bool enabledDrive = false;
volatile bool enabledTurn = false;
volatile bool enabledHunting = false;
volatile bool enabledMicArray = true;

CRGB leds[NUM_LEDS]; 

// Definition of callback functions for different states
typedef void (*voidFunction) (void); 
voidFunction callbackStateFunction;
voidFunction callbackDriveStateFunction;


void setup(){
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    Application_FunctionSet.ApplicationFunctionSet_Init();

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
      stateTimer.reset(); // to start the timer again
    }

    if (driveTimer.hasTimedOut()){ 
      // Set the LEDs depending on the current state
      if (currentState == Idle){
        leds[0] = CRGB::Green;
        FastLED.show();
      }
      if (currentState == Alerted){
        leds[0] = CRGB::Orange;
        FastLED.show();
      }
        
      enabledDrive = false; // To stop the roboter movement
      int driverTimeOutTime = stateTimer.getTimeOutTime() + 1000; //to guarantee that drivertimer does not timeout earlier than state
      //So we want to stop moving and starting the state timer again. The driver timer is set correctly when its needed (when the
      // roboter starts moving) in setDrivingStates
      driveTimer.setTimeOutTime(driverTimeOutTime);
      driveTimer.reset();
      stateTimer.reset();
    }

    // After each movement we want to wait 1sec until the mic array is activated again, because otherwise the
    // mics get a signal from somewhere (possibly from the servos) and interprets it as a noise
    if (enabledDrive == false && currentState != Init && stateTimer.getElapsedTime()>1000){
      enabledMicArray = true;
    }
    
    if (enabledDrive == true){
        callbackDriveStateFunction();
    }else{
        //To ensure that roboter first turns and than drives forward
        if (currentDrivingState == Turning){
            setDrivingState(Forward);
        }else if(currentDrivingState == Forward){
            //Hunting ends, enemy caught
            setDrivingState(DoNothing);
            //Make a sound
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

//For noise detection by mic array ###############
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
    if(currentState != Init){
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

//State machine #################
void setRobiState(RobiStates newState){
    if (currentState != newState){
        switch (newState)
            {
            case Init:
                leds[0] = CRGB::Blue;
                FastLED.show();
                timerThreshold = 15000; //15sec
                stateTimer.setTimeOutTime(timerThreshold);
                stateTimer.reset();
                callbackStateFunction = []() -> void {
                    attachInterrupt(digitalPinToInterrupt(pirPin), pirISR, CHANGE);
                    enabledMicArray = true;
                    setRobiState(Idle);
                };
                break;
            case Idle:
                digitalWrite(LED_BUILTIN, HIGH);
                leds[0] = CRGB::Green;
                FastLED.show();
                timerThreshold = 10000; //Move every 10 sec
                stateTimer.setTimeOutTime(timerThreshold);
                stateTimer.reset();
                callbackStateFunction = []() -> void{
                    //move around random
                    setDrivingState(Random);
                };
                break;
            case Alerted:
                leds[0] = CRGB::Orange;
                FastLED.show();
                timerThreshold = 5000; //Move every 4 sec
                stateTimer.setTimeOutTime(timerThreshold);
                stateTimer.reset();
                timeInAlerted = 0;
                callbackStateFunction = []() -> void{
                    // move around faster
                    setDrivingState(Random);
                    timeInAlerted = timeInAlerted + timerThreshold;
                    if (timeInAlerted > alertedTimeThreshold){
                        setRobiState(Idle);
                    }
                };
                
                break;
            case Hunting:
                // catch & follow the sound
                leds[0] = CRGB::Red;
                FastLED.show();
                setDrivingState(Turning);
                callbackStateFunction = []() -> void{
                  //No need for defining a callback, because there are just 2 steps to do for hunting: turning, driving
                  //And if there is a new signal afterwards, it will trigger the state machine again
                };
                
                break;
            default:
                break;
            }
        currentState = newState;
    }
}

// Driving states, so that different movement actions are done separately
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
                    //Do Obstacle Avoidance
                    Application_FunctionSet.ApplicationFunctionSet_Obstacle();
                };
                
                break;
            case Forward:
                //Calculate timer threshold depending on the forward distance
                timerDriveThreshold = getForwardTimeThreshold(getForwardDistance());
                Serial.println("Forward Mode");
                driveTimer.setTimeOutTime(timerDriveThreshold);
                driveTimer.reset();
                enabledDrive = true;
                enabledMicArray = false;
                callbackDriveStateFunction = []() -> void {
                    Application_FunctionSet.ApplicationFunctionSet_DriveRobi(speed);
                };
                break;
      
            case Turning:
                //Calculate timer threshold depending on turn angle
                timerDriveThreshold = getTurnTimerThreshold(getTurnAngle());
                Serial.print("Angle Threshold: ");
                Serial.println(timerDriveThreshold);
                driveTimer.setTimeOutTime(timerDriveThreshold);
                driveTimer.reset();
                enabledDrive = true;
                enabledMicArray = false;
                callbackDriveStateFunction = []() -> void{
                    int direction = TURN_RIGHT;
                    Serial.print("Angle direction: ");
                    Serial.println(getTurnAngle());
                    //If negative angle, turn left
                    if(getTurnAngle()<0){
                        direction = TURN_LEFT;
                    }
                   Application_FunctionSet.ApplicationFunctionSet_TurnRobi(direction, speed); 
                };             
                break;
            case DoNothing:
                //Stop the engines
                enabledDrive = false;            
                Application_FunctionSet.ApplicationFunctionSet_StopRobi();
                break;
            default:
                break;
            }
        currentDrivingState = newState;   
}

//Handling input from the PIR
void pirISR(){
    int pirStat = digitalRead(pirPin);
    if ((currentState == Idle) && (pirStat == HIGH)){
        setRobiState(Alerted);
    }else if ((currentState == Alerted) && (pirStat == HIGH)){
        timeInAlerted = 0;
    }    
}


//Calculate timer threshold depending on the turn angle. This method is calibrated for speed=100
int getTurnTimerThreshold(int turnAngle){
  int threshold = 0;
  if (speed > 75 && speed < 125){
    int offset = 100; // offset to get car into motion for speed 100 with turn of ~10 degree
    static float k = 8;
    static float threshold_float = float(turnAngle - 10) * k;
    return threshold = offset + int(threshold_float);
  }
  else{
    return threshold;
  }
}

// Calculate time threshold depending on the distance in cm. This method is calibrated for speed=100
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


