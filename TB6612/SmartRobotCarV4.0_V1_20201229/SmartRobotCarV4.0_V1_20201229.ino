#include <avr/wdt.h>
#include "ApplicationFunctionSet_xxx0.h"
#include <SoftTimers.h>
//#include <TimerOne.h>
#include <TimerFive.h>
#include <arduino.h>
#include <TimerThree.h>
#include <FastLED.h>
#include <TimerFour.h>	

enum RobiStates{
    Idle,
    Alerted,
    Hunting,
    Init
};
 

const int pirPin = 2;
const int alertedTimeThreshold = 120;
const int PIN_RBGLED = 4;
const int NUM_LEDS = 1;

// könnte man als enum machen, habe ich nur nicht direkt hinbekommen weil das in der Funktion DeviceDriverSet_xxx0.cpp verwendet wird
const int TURN_LEFT = 0; 
const int TURN_RIGHT = 1;

SoftTimer stateTimer;
SoftTimer driveTimer;
SoftTimer turnTimer; // maybe not needed -> depends on states propably

RobiStates currentState;
int timerCounter = 0;
int timeInAlerted = 0;
int timerThreshold = 5;
int timerDriveCounter = 0;
int timerDriveTreshold = 5000;
int timerTurnTreshold = 2000;

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


void setup(){
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    Application_FunctionSet.ApplicationFunctionSet_Init();

    stateTimer.setTimeOutTime(timerThreshold);
    stateTimer.reset();

    //Timer1.initialize(1000000);
    //Timer1.attachInterrupt(timerISR);
    //Timer5.initialize(100000);
    //Timer5.attachInterrupt(timerDriveISR);
    FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
    FastLED.setBrightness(200);
    initRobi();
}

void loop(){
    if (stateTimer.hasTimedOut()){
      digitalWrite(LED_BUILTIN, HIGH);
      callbackStateFunction();
    }
    if (driveTimer.hasTimedOut()){
      enabledDrive = false;
      Serial.println("Driver timed out");
    }
    if (turnTimer.hasTimedOut()){
      enabledTurn = false;
      Serial.println("Turn timed out");
    }

    if (enabledTurn == true){
      leds[0] = CRGB::Red;
      FastLED.show();
      Application_FunctionSet.ApplicationFunctionSet_TurnRobi(TURN_LEFT, speed);
    }else{
      Serial.println("Else STOP Turn");
      Application_FunctionSet.ApplicationFunctionSet_StopRobi();        
    }

    // wie bereits gesagt @Joshua: Zusammensoiel von beiden Funktionen muss noch geklärt werden, damit sie nicht "gleichzeitig" ausgeführt werden
    /*if(enabledDrive == true){
        
        leds[0] = CRGB::Red;
        FastLED.show();
        Application_FunctionSet.ApplicationFunctionSet_DriveRobi(speed);
      }else{
        //leds[0] = CRGB::Yellow;
        //FastLED.show();
        Serial.println("Else STOP");
        Application_FunctionSet.ApplicationFunctionSet_StopRobi();
    }*/
}

void initRobi(){
    setRobiState(Init);
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
                timerThreshold = 1500; //120
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
                timerThreshold = 3000; //300
                stateTimer.setTimeOutTime(timerThreshold);
                stateTimer.reset();

                // define turnAngle and get accordingly timerTurnTreshold
                turnAngle = 90; // future: will be defined by sensor
                timerTurnTreshold = getTurnTimerTreshold();

                // MH: define meters (global variable) to drive forward
                // MH: define speed (global variable) to drive with 
                // MH:call function to calculate timerDriveTreshold (similar to getTurnTimerTreshold())

                callbackStateFunction = []() -> void{
                    //move around random
                    //Timer5.restart();
                    turnTimer.setTimeOutTime(timerTurnTreshold);
                    turnTimer.reset();
                    enabledTurn = true;
                    driveTimer.setTimeOutTime(timerDriveTreshold);
                    driveTimer.reset();
                    enabledDrive = true;
                    stateTimer.setTimeOutTime(timerThreshold);
                    stateTimer.reset();
                    
                    
                    /*while (enabled == true){
                       Application_FunctionSet.ApplicationFunctionSet_Obstacle(); 
                       leds[0] = CRGB::Yellow;
                       FastLED.show();
                    }*/
                    
                };
                break;
            case Alerted:
                timerThreshold = 60;
                timeInAlerted = 0;
                callbackStateFunction = []() -> void{
                    // move around faster
                    timeInAlerted = timeInAlerted + timerThreshold;
                    if (timeInAlerted > alertedTimeThreshold){
                        setRobiState(Idle);
                    }
                };
                
                break;
            case Hunting:
                // catch & follow the sound
                callbackStateFunction = []() -> void{
                    
                };
                timerThreshold = 1;
                break;
            default:
                break;
            }
        currentState = newState;
    }
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
int getTurnTimerTreshold(){
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
/*
void timerISR(){
    timerCounter = timerCounter + 1;
    if (timerCounter > timerThreshold){
        timerCounter = 0;
        digitalWrite(LED_BUILTIN, HIGH);
        callbackStateFunction();
        //enabled = true;
    }
}

void timerDriveISR(){
    timerDriveCounter = timerDriveCounter + 1;
    //enabled = true;
    if (timerDriveCounter > timerDriveTreshold){
        timerDriveCounter = 0;
        enabledDrive = false;
    }
}
*/
