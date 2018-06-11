#include <Wire.h>
#include <Servo.h>
#include <Timer.h>
#include <LSM303.h>
#include <EEPROM.h>

#include <math.h>
#include "State.h"
Timer t;

Timer tc;
int coun1;
int coun2;
State state;
void setup() {
  Serial.begin(9600);
  Wire.begin();
  state.reader.init (9);
  pinMode(2,INPUT);
  pinMode(3,INPUT);
  t.every(100,takeReading);
  
  //tc.every(20,sendCompass);
  attachInterrupt(0,pin_ISR1,RISING);
  attachInterrupt(1,pin_ISR2,RISING);
  coun1=coun2 = 0;
  Serial.println("START");
}
void pin_ISR1() {
  coun1++;
}

void pin_ISR2() {
  coun2++;
}
void takeReading()
{
  state.set_v1 (coun1*10.0/20);
  
  state.set_v2 (coun2*10.0/20);

  state.change_state ();
}

void sendCompass()
{
  state.sendCompass();
}
void loop() {
  t.update();
  tc.update();
  state.update_state();
 // Serial.println(millis()/1000);
 // Serial.println(state.get_x());
 // Serial.println(state.get_y());
  Serial.println(state.get_orientation());
  Serial.println("");
 // Serial.println(v1);
  
  //Serial.println(v2);
//  /delay (400);
}
