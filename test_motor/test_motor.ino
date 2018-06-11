
#include <PID_v1.h>
#include <math.h>
#include "Motor_Driver.h"

#include <Wire.h>

#define left_motor 3
#define right_motor 1

struct point  {
  double x; 
  double y;
};
double heading; //Robot heading received from serial
struct point location; // Robot Location receive from serial
struct point target; //Target to reah received from serial

struct point ttarget; //Target to reah received from serial
struct point initial;
double v1,v2; //current velocity of Robot received from serial
double tv1,tv2; // target verlocity of Robot (constant 5 cm/s)
double pv1,pv2; //provided velocity to robot (converted to pwm)
double temp;
double *holder[8];
double Kp=2, Ki=5, Kd=1;  //PID constants measured by tuning libraries
PID PID1(&v1, &pv1, &tv1, Kp, Ki, Kd, DIRECT);
PID PID2(&v2, &pv2, &tv2, Kp, Ki, Kd, DIRECT);

int get_int() {
  
  int x1 = Wire.read();
  x1=x1<<8;
  x1|=Wire.read();
  return x1;
}
long startt;
int speed_to_pwm(double speed) {
  return min(speed*128/5.0,255);
}

void Move(int motr,double speed,int dir){
  if(speed == 0.0){
    motor(motr,BRAKE,0);
    return;
  }
  switch(motr){
    case left_motor:
    tv1 = speed;

    PID1.Compute();
   // Serial.print("vp1 is ");
   // Serial.println(pv1);
    motor(left_motor, dir, speed_to_pwm(tv1));
    break;
    case right_motor:
    
    tv2 = speed;
    PID2.Compute();
    
    motor(right_motor, dir, speed_to_pwm(tv2));
    break;
  }
}

void setup()
{
  target.x = target.y = -1;
  Serial.begin(9600);  
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
           // start serial for output
           startt = millis();
 PID1.SetMode(AUTOMATIC);
 
  PID2.SetMode(AUTOMATIC);
  holder[0]  = &v1;
  
  holder[1]  = &v2;
  
  holder[2]  = &location.x;
  
  holder[3]  = &location.y;
  
  holder[4]  = &heading;
  
  holder[5]  = &target.x;
  
  
  holder[6]  = &target.y;
  
  holder[7]  = &temp;
  ttarget.x = 5;
  ttarget.y = 50;
   delay(2000);
    initial = location;
           Serial.println("STARTING");
}
// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  if(Wire.available() < 3)
    return;
    int i = Wire.read()-'a';
 int he=get_int();
  *holder[i]=he/100.0;
if(i==2||i==3){
//    Serial.println("loc received");
    Serial.println(he);
  } 
  
}
bool alligned = 0;
bool reached = 0;
void loop()
{
  
 // move_to_target();
  
    //motor(left_motor, FORWARD, 128);
    //motor(right_motor, FORWARD, 128);
 //  Move(left_motor, 2.5,FORWARD);
   
  // Move(right_motor, 2.5,FORWARD);
//  Serial.println(location.x);
//
//  Serial.println(*holder[2]);
//  
//  Serial.println(*holder[3]);
//  Serial.println(location.y);
//  
//  if(!alligned)
//    allign(initial,target);
//    else if(!reached)
//    move_to_target();
    
}
/**
 * Function responsible for moving to given target
 */
void move_to_target(){
  if(fabs(target.x-location.x) < 10 && fabs(target.y-location.y) < 10) {
       Move(left_motor,0.0,FORWARD);
        Move(left_motor,0.0,FORWARD);
        reached = 1;
        return;
   
  }
  double const_speed = 3.5;
  float slope = target.y-location.y;
  slope/=1;
  
  float angle =atan2(slope,(target.x-location.x))*180/3.14159; 
   if(fabs(angle - heading) > 15) {
     // allign(location,target);
   }
   Move(left_motor,const_speed,FORWARD);
   Move(right_motor,const_speed,FORWARD);
   
}


/**
 * allign robot to walk in straight line 
 */
void allign(struct point me,struct point target) {
  float slope = target.y-me.y;
  float angle =atan2(slope,(target.x-location.x))*180/3.14159; 
  allign_angle(angle);
  
}
int tspeed = 80;
void allign_angle(double  angle) {
  if(angle - heading > 5) { //if error in angle is > tolerance 
    
    motor(left_motor, FORWARD, tspeed);
    motor(right_motor, BACKWARD, tspeed);
    delay(10);
    
  }
  else if (angle - heading <-5) { //if error in angle is > tolerance 
     motor(left_motor, BACKWARD, tspeed);
    motor(right_motor, FORWARD, tspeed);
    
    delay(10);
  }
  else {
        motor(left_motor, RELEASE, 0);
    motor(right_motor, RELEASE, 0);
    alligned = 1;
    delay(3000);
    
  }
        motor(left_motor, BRAKE, 0);
    motor(right_motor, BRAKE, 0);
}

