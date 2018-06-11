#include "Communication.h"
#include <Wire.h>
#include <Arduino.h>


Communication::Communication (){

}


void Communication::transmit_to_slave (float f,const char *message) {
  Wire.beginTransmission(8); // transmit to device #8
  int x = f*100;
  Wire.write(message);
  Wire.write((byte)(x>>8));
  Wire.write((byte)(x&255));
                // sends one byte
  Wire.endTransmission(); // stop transmitting

}


int Communication::receive_data () {
  int data = -1;
  if (Serial.available() > 1)
    data = Serial.parseInt();
  return data;
}
