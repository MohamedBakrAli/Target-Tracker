#include "State.h"
#define LENGTH 60
#define WIDTH 80
#define delayTime 400
State::State() {
  last_millis = millis();
  ang =0;
  targetX = 50;
  targetY = 50;
  }
  
void State::wait_while(long milli){
      long start = millis();
      while(millis()-start < milli) {
        orientation = reader.get_current_orientation();
        if(millis()-start)%100) {
          change_state();
        }
        else if((millis()-start)%20 == 0)
          sendCompass();
          
      }
  }
void State::update_state() {
  float newX, newY;
  double deltas = (millis() - last_millis) / 1000.0;
  last_millis = millis();

  if ((newX = communication.receive_data ()) != -1 && (newY = communication.receive_data ()) != -1)  //check for new headings
    targetX = newX, targetY = newY;

  double x1, x2, y1, y2;
  orientation = reader.get_current_orientation();
  //Serial.println(orientation);
  double actual[2];
  ang = orientation;
  float tang =0+ang;
      bool swish = 0;
        if(servo_ang !=tang) {
         if(tang<0 ) {
          Serial.print("Servo ang 1 is ");
          Serial.println(tang+180);
        reader.servo.write(tang+180);
          swish = 1;
        }
        else if(tang > 180) {
          
          Serial.print("Servo ang 1 is ");
          Serial.println(tang-180);
        reader.servo.write(tang-180);
          swish = 1; 
        }
        else {
          
          Serial.print("Servo ang 1 is ");
          Serial.println(tang);
        reader.servo.write(tang);
        swish = 0;
        }
        wait_while(delayTime);
        servo_ang= tang;
      }
  x1 = (swish)?reader.get_front_distance():LENGTH-reader.get_front_distance();
  x2 = (swish)?LENGTH-reader.get_rear_distance():reader.get_rear_distance();
   tang =90+ang;
   swish = 0;
      if(tang>180)
        tang-=360;
      if(servo_ang !=tang) {
         if(tang<0 ) {
          
         // Serial.print("Servo ang 2 is ");
        //  Serial.println(tang+180);
        reader.servo.write(tang+180);
          swish = 1;
        }
        else if(tang > 180) {
          
          //Serial.print("Servo ang 2 is ");
         // Serial.println(tang-180);
        reader.servo.write(tang-180);
          swish = 1; 
        }
        else {
          
//          Serial.print("Servo ang 2 is ");
  //        Serial.println(tang);
        reader.servo.write(tang);
        swish = 0;
        }
        wait_while(delayTime);
        servo_ang= tang;
      }

  y1 = (swish)?WIDTH-reader.get_left_distance():reader.get_left_distance();
  y2 = (swish)?reader.get_right_distance():WIDTH-reader.get_right_distance();
  Serial.print("X1 = ");
  Serial.print(x1);
  Serial.print(", X2 = ");
  Serial.print(x2);
  Serial.print(", Y1 = ");
  Serial.print(y1);
  Serial.print(", Y2 = ");
  Serial.println(y2);
  kalman.getFilteredValue(x1, y1, deltas, &actual[0]);
  kalman.getFilteredValue(x2, y2, 0.0, &actual[0]);
 
  x = actual[0];
  y = actual[1];
}

float State::get_x() {

  return x;
}

float State::get_y() {
  return y;
}

float State::get_target_x() {

  return targetX;
}

float State::get_target_y() {
  return targetY;
}

float State::get_orientation() {
  return orientation;
}
void State::sendCompass() {
  orientation = reader.get_current_orientation();
  communication.transmit_to_slave (reader.orient,"e");
}
void State::change_state () {
  
  communication.transmit_to_slave (v1,"a");
  communication.transmit_to_slave (v2,"b");
  communication.transmit_to_slave (x,"c");
  communication.transmit_to_slave (y,"d");
  communication.transmit_to_slave (orientation,"e");
  
  communication.transmit_to_slave (targetX,"f");
  communication.transmit_to_slave (targetY,"g");
  
  communication.transmit_to_slave (-20.5,"h");
}


float State::get_v1(void) {
  return v1;
}

float State::get_v2(void) {
  return v2;
}

void State::set_v1(float _v1) {
  v1 = _v1;
}

void State::set_v2(float _v2) {
  v2 = _v2;
}

