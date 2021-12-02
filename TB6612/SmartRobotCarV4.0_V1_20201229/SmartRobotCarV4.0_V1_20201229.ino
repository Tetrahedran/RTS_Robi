
#include <avr/wdt.h>
#include "ApplicationFunctionSet_xxx0.h"

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

RobiStates currentState;
int timerCounter = 0;
int timeInAlerted = 0;
int timerThreshold = 5;

bool enabled = true;

CRGB leds[NUM_LEDS]; 

typedef void (*voidFunction) (void); 
voidFunction callbackStateFunction;


void setup(){
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    Application_FunctionSet.ApplicationFunctionSet_Init();
    Timer5.initialize(1000000);
    Timer5.attachInterrupt(timerISR);
    FastLED.addLeds<NEOPIXEL, PIN_RBGLED>(leds, NUM_LEDS);
    FastLED.setBrightness(200);
    initRobi();
}

void loop(){
    if(enabled == false){
        leds[0] = CRGB::Red;
        FastLED.show();
        Application_FunctionSet.ApplicationFunctionSet_Obstacle();
    }else{
        leds[0] = CRGB::Yellow;
        FastLED.show();

    }
   //Application_FunctionSet.ApplicationFunctionSet_Obstacle();
    
}

void initRobi(){
    //setRobiState(Init);
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
                timerThreshold = 120;
                callbackStateFunction = []() -> void {
                    attachInterrupt(digitalPinToInterrupt(pirPin), pirISR, CHANGE);
                    setRobiState(Idle);
                };
                break;
            case Idle:
                digitalWrite(LED_BUILTIN, HIGH);
                leds[0] = CRGB::Green;
                FastLED.show();
                timerThreshold = 300;
                callbackStateFunction = []() -> void{
                    //move around random
                    Application_FunctionSet.ApplicationFunctionSet_Obstacle();
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
        //digitalWrite(LED_BUILTIN, HIGH);
        //callbackStateFunction();
        enabled = false;
    }
}

