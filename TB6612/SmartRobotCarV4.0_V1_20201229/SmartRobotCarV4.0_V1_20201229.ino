

#include <avr/wdt.h>
#include "ApplicationFunctionSet_xxx0.h"
#include <SoftTimers.h>
#include <TimerOne.h>
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

SoftTimer stateTimer;
SoftTimer driveTimer;

RobiStates currentState;
int timerCounter = 0;
int timeInAlerted = 0;
int timerThreshold = 5;
int timerDriveCounter = 0;
int timerDriveTreshold = 5000;

volatile bool enabled = false;

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
        enabled = false;
        Serial.println("Driver timed out");
    }

    if(enabled == true){
        
        leds[0] = CRGB::Red;
        FastLED.show();
        Application_FunctionSet.ApplicationFunctionSet_Obstacle();
    }else{
        //leds[0] = CRGB::Yellow;
        //FastLED.show();
        Serial.println("Else STOP");
        Application_FunctionSet.ApplicationFunctionSet_StopRobi();

    }
   //Application_FunctionSet.ApplicationFunctionSet_Obstacle();
    
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
                timerThreshold = 15000; //120
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
                timerThreshold = 30000; //300
                stateTimer.setTimeOutTime(timerThreshold);
                stateTimer.reset();
                callbackStateFunction = []() -> void{
                    //move around random
                    //Timer5.restart();
                    driveTimer.setTimeOutTime(timerDriveTreshold);
                    driveTimer.reset();
                    enabled = true;
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
        enabled = false;
    }
}

