#include <avr/wdt.h>
#include <arduino.h>
#include <arduino-timer.h>
#include <TimerThree.h>

enum RobiStates{
    Idle,
    Alerted,
    Hunting
};

int pirPin = 2;
int pirStat = 0;

RobiStates currentState;
int timerCounter = 0;


void setup(){
    init_Robi();
    Timer1.init(1000000);
    Timer1.attachInterrupt(timerISR);
    pinMode(LED_BUILTIN, OUTPUT);
    noInterrupts();
    TCCR0A = 0;
    TCCE0B = 0;
    TCNT0 = 0;

    OCR0A = 255;
    TCCR0B |= (1 << WGM02)
    TCCR0B |= (1 << CS02) | (1 << CS00)
    TIMSK0 |= (1 << OCIE0A);
    interrupts();
    
}

void loop(){
    switch (currentState)
    {
    case Idle:
        /* code */
        digitalWrite(LED_BUILTIN, LOW);
        if (pirStat == HIGH){
            currentState = Alerted;
        }
        break;
    case Alerted:
        /* code */
        digitalWrite(LED_BUILTIN, HIGH);
        if (pirStat == LOW){
            currentState = Idle;
        }
        break;
    case Hunting:
        /* code */
        break;
    default:
        break;
    }
}

void init_Robi(){
    currentState = Idle;
    pinMode(pirPin, INPUT);
    Serial.begin(9600);
    attachInterrupt(digitalPinToInterrupt(pirPin), pirISR, CHANGE);
}

void pirISR(){
    pirStat = digitalRead(pirPin);
}

void timerISR(){
    timerCounter = timerCounter + 1;
}

ISR(TIMER0_COMPA_vect){
    
    timerCounter = timerCounter + 1;
}