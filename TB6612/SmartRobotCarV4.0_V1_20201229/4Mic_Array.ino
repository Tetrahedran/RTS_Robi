#include <stdlib.h>


const int pins[] = {A1, A2, A3, A4};
const int dir[] = {45, 135, 225, 315};
//float calib[] = {1.0f,1.43f,1.0f,2.22f};
float calib[] = {1,1,1,1};
const int pin_size = 4;
const int threshold = 70;

bool nd = false;
int cnt[pin_size][2];


void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  for(int i = 0; i < pin_size; i++){
    pinMode(pins[i], INPUT);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
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
