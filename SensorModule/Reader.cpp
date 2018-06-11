#include "Arduino.h"
#include "Reader.h"
#include "Anything.h"
#define delayTime 400
// TODO add bluetooth module

void StateReader::init (int servo_pin) {
  servo_ang = 0;
  ang = 0;
  servo.attach(servo_pin);
  servo.write(0);
  compass.init();
  compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>) {
    -32767, -32767, -32767
  };
  compass.m_max = (LSM303::vector<int16_t>) {
    +32767, +32767, +32767
  };
  
  get_compass_readings ();
  long start = millis();
  
  Serial.println ("start");

  relative_east = calculate_heading();
}

void StateReader::get_compass_readings () {
  int x;
  EEPROM_readAnything (0, x);
  compass.m_min.x = x;
  EEPROM_readAnything (2, x);
  compass.m_min.y = x;
  EEPROM_readAnything (4, x);
  compass.m_min.z = x;
  EEPROM_readAnything (6, x);
  compass.m_max.x = x;
  EEPROM_readAnything (8, x);
  compass.m_max.y = x;
  EEPROM_readAnything (10, x);
  compass.m_max.z = x;
}

StateReader::StateReader(int left_trig, int left_echo, int right_trig, int right_echo) {
  this->left_trig = left_trig;
  this->left_echo = left_echo;
  this->right_trig = right_trig;
  this->right_echo = right_echo;

}

double StateReader::get_front_distance() {
      

  return calculate_distance(calculate_time(left_trig, left_echo));
}


double StateReader::get_rear_distance() {

  return calculate_distance(calculate_time(right_trig, right_echo));
}

double StateReader::get_right_distance() {

  return calculate_distance(calculate_time(right_trig, right_echo));
}

double StateReader::get_left_distance() {
  return calculate_distance(calculate_time(left_trig, left_echo));
}

float StateReader::get_current_orientation() {
  orient = calculate_heading() - relative_east;
  if (orient < -180)
    orient += 360;
  if (orient > 180)
    orient -= 360;
    ang = (int)(orient/2);
    ang*=2;
    ang=0;
  return orient;
}

long StateReader::calculate_time(int trig_pin, int echo_pin) {
  pinMode(trig_pin, OUTPUT);
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(trig_pin, LOW);

  pinMode(echo_pin, INPUT);
  return pulseIn(echo_pin, HIGH);
}

long StateReader::calculate_distance(long times) {
  /* The speed of sound is 340 m/s or 29 microseconds per centimeter.
     The ping travels out and back, so to find the distance of the
     object we take half of the distance travelled. */
  return times  / 29 / 2;
}

float StateReader::calculate_heading() {
  compass.read();
  float heading = compass.heading();
  return heading;
}
