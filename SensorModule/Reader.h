#ifndef READER_H
#define READER_H

#include <Arduino.h>
#include <Wire.h>

#include <LSM303.h>

#include <Servo.h>


class StateReader {

private:
  
  LSM303 compass;
    int left_trig,  left_echo;
    int right_trig, right_echo;
//    sensors_event_t event;
    float relative_east;
    float ang;
    int servo_ang;

    long calculate_distance(long times);
    long calculate_time(int trig_pin, int echo_pin);

    float calculate_heading(void);
    void get_compass_readings (void);
public:

Servo servo;
   StateReader(int left_trig, int left_echo, int right_trig, int right_echo);

    /* each function reads the value measured by one of the sensors
   * returns  a double (d) representing the distance */
    double get_front_distance();

    double get_rear_distance();

    double get_right_distance();

    double get_left_distance();

    /* reads the value measured by the compass
     * returns an int (angle) representing the current compass orientation */
    float get_current_orientation();
    double orient;
    void init (int);


};


#endif
