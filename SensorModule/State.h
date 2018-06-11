#ifndef STATE_H
#define STATE_H

#include <Arduino.h>
#include "kalman.h"
#include "Reader.h"
#include "Communication.h"
class State {

private:

    
    float x, y;
    float v1,v2;
    float orientation;
    long last_millis;
    float targetX, targetY;
    Communication communication;
    Kalman kalman;
    double ang;
    double servo_ang;
    void wait_while(long milli);

public:

    StateReader reader = StateReader(8,10,12,11);
    State();
    void update_state(void);

    float get_x(void);

    float get_y(void);

    float get_v1(void);

    float get_v2(void);

    void set_v1(float);

    void set_v2(float);

    float get_target_x (void);

    float get_target_y (void);

    float get_orientation(void);

    void change_state (void);
    
    void sendCompass(void);
};


#endif //MICRO_STATE_H
