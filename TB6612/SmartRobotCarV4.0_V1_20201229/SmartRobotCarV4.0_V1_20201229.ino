#include <avr/wdt.h>
#include "ApplicationFunctionSet_xxx0.h"
#include <SoftTimers.h>
#include <arduino.h>

#include <FastLED.h>


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

// könnte man als enum machen, habe ich nur nicht direkt hinbekommen weil das in der Funktion DeviceDriverSet_xxx0.cpp verwendet wird
const int TURN_LEFT = 0; 
const int TURN_RIGHT = 1;

SoftTimer stateTimer;
SoftTimer driveTimer;
SoftTimer turnTimer; // maybe not needed -> depends on states propably

RobiStates currentState;
DrivingStates currentDrivingState;
int timerCounter = 0;
int timeInAlerted = 0;
int timerThreshold = 5;
int timerDriveCounter = 0;
int timerDriveThreshold = 5000;
int timerTurnThreshold = 2000;

int testTimeInIdle = 0;

//initilize car speed for initial mode
uint8_t speed = 100;
// initialize meters to drive formard in cm --> not used in current version 
int driveForwardDistance = 500; 
int turnAngle = 90; // 90 degree

volatile bool enabledDrive = false;
volatile bool enabledTurn = false;

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
}

void loop(){
  //Serial.println(timerDriveThreshold);
    if (stateTimer.hasTimedOut()){
      digitalWrite(LED_BUILTIN, HIGH);
      callbackStateFunction();
      stateTimer.reset();
      //Serial.println(callbackStateFunction == nullptr);
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
      int driverTimeOutTime = stateTimer.getTimeOutTime() + 1000;
      driveTimer.setTimeOutTime(driverTimeOutTime);
      driveTimer.reset();
      stateTimer.reset();
      //Serial.println("Driver timed out");
    }
    
    //Serial.println(driveTimer.getElapsedTime());
    
    if (enabledDrive == true){
        callbackDriveStateFunction();
    }else{
        setDrivingState(DoNothing);
      Application_FunctionSet.ApplicationFunctionSet_StopRobi();   
    }
    
}

void initRobi(){
    setRobiState(Init);
    setDrivingState(DoNothing);
    //currentDrivingState = DoNothing;
    pinMode(pirPin, INPUT);
    Serial.begin(9600);
    
}

void setRobiState(RobiStates newState){
    if (currentState != newState){
        switch (newState)
            {
            case Init:
                leds[0] = CRGB::Blue;
                FastLED.show();
                timerThreshold = 10000; //10sec
                stateTimer.setTimeOutTime(timerThreshold);
                stateTimer.reset();
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

                callbackStateFunction = []() -> void{
                    //move around random
                    Serial.println("Exec Random");
                    setDrivingState(Random);
                    testTimeInIdle = testTimeInIdle + 1;
                    if (testTimeInIdle>2){
                        setRobiState(Alerted);
                    }
                };
                break;
            case Alerted:
                leds[0] = CRGB::Orange;
                FastLED.show();
                timerThreshold = 5000; //alle 5 sec bewegen
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
                callbackStateFunction = []() -> void{
                    //set variables, direction of sound, setDrivingState(Turning), setDrivingState(Forward)
                };
                timerThreshold = 1;
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
                
                callbackDriveStateFunction = []() -> void {
                    Application_FunctionSet.ApplicationFunctionSet_Obstacle();
                };
                enabledDrive = true;
                break;
            case Forward:
                // MH: define meters (global variable) to drive forward
                // MH: define speed (global variable) to drive with 
                // MH:call function to calculate timerDriveTreshold (similar to getTurnTimerTreshold())
                //timerDriveTreshold = 4000; //more like calc it
                int forwardDistance = 53;
                timerDriveThreshold = getForwardTimeThreshold(forwardDistance);
                Serial.println("Forward Mode");
                Serial.println(timerDriveThreshold);
                driveTimer.setTimeOutTime(timerDriveThreshold);
                driveTimer.reset();
                enabledDrive = true;
                callbackDriveStateFunction = []() -> void {
                    Application_FunctionSet.ApplicationFunctionSet_DriveRobi(speed);
                };
                break;
      
            case Turning:
                // define turnAngle and get accordingly timerTurnTreshold
                turnAngle = 90; // future: will be defined by sensor
                timerDriveThreshold = getTurnTimerThreshold();
                driveTimer.setTimeOutTime(timerDriveThreshold);
                driveTimer.reset();
                enabledDrive = true;
                callbackDriveStateFunction = []() -> void{
                   Application_FunctionSet.ApplicationFunctionSet_TurnRobi(TURN_LEFT, speed); 
                };
                
                break;
            case DoNothing:
                //
                enabledDrive = false;
                //leds[0] = CRGB::Green;
                //    FastLED.show();
                Application_FunctionSet.ApplicationFunctionSet_StopRobi();
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
int getTurnTimerThreshold(){
  int treshold = 0;
  if (speed > 75 && speed < 125){
    int offset = 100; // offset to get car into motion for speed 100 with turn of ~10 degree
    static float k = 8;
    static float treshold_float = float(turnAngle - 10) * k;
    return treshold = offset + int(treshold_float);
  }
  // weitere Speed bereiche definieren --> k & offset ...
  else{
    return treshold;
  }
}


int getForwardTimeThreshold(int pathLength){
    int threshold = 0;
    if (speed > 75 && speed < 125){
        if (pathLength<=20){
            static int k = 50;
            return threshold = pathLength * k;
        }else{
            float adjustPath = pathLength - 20;
            float calibrate = adjustPath / 40;
            float readjustPath = calibrate + 1;
            return threshold = readjustPath * 1000;
        }
    }else{
        return threshold;
    }
}
