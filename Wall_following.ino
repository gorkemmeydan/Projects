/************************************************
 * This is the code for the control of a pallated unmanned 
 * land vehicle in a maze.
 * 
 * Görkem Meydan 2018
 * **********************************************/

#include <NewPing.h>

#define SONAR_NUM 3      // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.

NewPing sonar[SONAR_NUM] = {  // Sensor object array.
  NewPing(6, 6, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(A0, A0, MAX_DISTANCE),
  NewPing(A1, A1, MAX_DISTANCE)
};

#define M1_ENCODER 2
#define M2_ENCODER 3
#define M1_IN1 7
#define M1_IN2 8
#define M1_PWM 10
#define M2_IN1 4
#define M2_IN2 5
#define M2_PWM 9

#define MAX_PWM 200
#define ULTRASONIC_MAX_PWM 200

/* Function prototypes */
void motor_right(unsigned char motor_speed, bool motor_direction);
void motor_left(unsigned char motor_speed, bool motor_direction);
void leftEncoderEvent(void);
void rightEncoderEvent(void);
void driveDistance(int);
void driveDistance_withUltrasonic(int cm_distance);
void followWallFromBothSides(int pwm);

// Moves in desired direction
// L: turn left 90 deg
// R: turn right 90 deg
// B: turn right 180 deg
// C: center the robot on the line
// P: just pass the white line
void go(char dir);

void stopRobot(void);
void goStraight(int pwm, int cycle_time);

// follow wall from both sides PID parameters
float _Kp = 0.5;
float _Ki = 0;
float _Kd = 0.1;
float _error = 0;
float _proportional, _derivative, _integral, _last_proportional;

// go straight PID parameters
float Kp_ = 3;
float Ki_ = 0;
float Kd_ = 0;
float error_ = 0;
float proportional_, derivative_, integral_, last_proportional_;

volatile unsigned long leftCount = 0;
volatile unsigned long leftPeriod = 0;
volatile unsigned long leftPrevTime = 0;

volatile unsigned long rightCount = 0;
volatile unsigned long rightPeriod = 0;
volatile unsigned long rightPrevTime = 0;

int encoderTicks = 0;

int normal_pwm = 100, left_pwm, right_pwm;

void setup() {
  pinMode(M1_IN1,OUTPUT);
  pinMode(M1_IN2,OUTPUT);
  pinMode(M1_PWM,OUTPUT);
  pinMode(M2_IN1,OUTPUT);
  pinMode(M2_IN2,OUTPUT);
  pinMode(M2_PWM,OUTPUT);

  pinMode(M2_ENCODER, INPUT_PULLUP);
  pinMode(M1_ENCODER, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(M2_ENCODER), leftEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M1_ENCODER), rightEncoderEvent, CHANGE);
  
}

void loop() {
  

}

// Speed: from 0 to 255
// Direction: 1:Forward, 0:Backward
void motor_right(unsigned char robot_speed, bool robot_direction)
{
 if(!robot_direction){
  digitalWrite(M1_IN1,LOW);
  digitalWrite(M1_IN2,HIGH);
  }
 else
 {
  digitalWrite(M1_IN1,HIGH);
  digitalWrite(M1_IN2,LOW);
 }
 analogWrite(M1_PWM,robot_speed);
}

void motor_left(unsigned char robot_speed, bool robot_direction)
{
 if(!robot_direction){
  digitalWrite(M2_IN1,LOW);
  digitalWrite(M2_IN2,HIGH);
  }
 else
 {
  digitalWrite(M2_IN1,HIGH);
  digitalWrite(M2_IN2,LOW);
 }
 analogWrite(M2_PWM,robot_speed);
}

void stopRobot(void)
{
  motor_left(0,1);
  motor_right(0,1);
}

// Moves in desired direction
// L: turn left 90 deg
// R: turn right 90 deg
// B: turn right 180 deg
// C: center the robot on the line
// P: just pass the white line
void go(char dir)
{

  switch(dir){

  case 'L':
    rightCount=0;
    leftCount=0;
    encoderTicks = 130 * 3;
    while (leftCount < encoderTicks || rightCount < encoderTicks)
    {
      if (leftCount < encoderTicks)
        motor_left(100, 0);
      if (rightCount < encoderTicks)
        motor_right(100, 1);
    }
    stopRobot();
    leftCount = 0;
    rightCount = 0;
    encoderTicks = 0;
    break;

  case 'R':
    rightCount=0;
    leftCount=0;
    encoderTicks = 130 * 3;
    while (leftCount < encoderTicks || rightCount < encoderTicks)
    {
      if (leftCount < encoderTicks)
        motor_left(100, 1);
      if (rightCount < encoderTicks)
        motor_right(100, 0);
    }
    stopRobot();
    leftCount = 0;
    rightCount = 0;
    encoderTicks = 0;
    break;

  case 'B':
    rightCount=0;
    leftCount=0;
    encoderTicks = 850;
    while (leftCount < encoderTicks || rightCount < encoderTicks)
    {
      if (leftCount < encoderTicks)
        motor_left(100, 1);
      if (rightCount < encoderTicks)
        motor_right(100, 0);
    }
    stopRobot();
    leftCount = 0;
    rightCount = 0;
    encoderTicks = 0;
      break;

  case 'C':
  rightCount=0;
  leftCount=0;
  encoderTicks = 280;
  while (leftCount < encoderTicks || rightCount < encoderTicks)
  {
    if (leftCount < encoderTicks)
      motor_left(100, 1);
    if (rightCount < encoderTicks)
      motor_right(100, 1);
  }
  stopRobot();
  leftCount = 0;
  rightCount = 0;
  encoderTicks = 0;
    break;

  case 'P':
  rightCount=0;
  leftCount=0;
  encoderTicks = 140;
  while (leftCount < encoderTicks || rightCount < encoderTicks)
  {
    if (leftCount < encoderTicks)
      motor_left(100, 1);
    if (rightCount < encoderTicks)
      motor_right(100, 1);
  }
  stopRobot();
  leftCount = 0;
  rightCount = 0;
  encoderTicks = 0;
    break;
  }
}

void leftEncoderEvent(void)
{
  leftCount++;
  //Serial.print("Left count: ");
  //Serial.println(leftCount);

  //leftPeriod = micros() - leftPrevTime;
  //Serial.print("Left period: ");
  //Serial.println(leftPeriod);
  //leftPrevTime = micros();
}

void rightEncoderEvent(void)
{
  rightCount++;
  //Serial.print("Right count: ");
  //Serial.println(rightCount);

  //rightPeriod = micros() - rightPrevTime;
  //Serial.print("Right period: ");
  //Serial.println(rightPeriod);
  //rightPrevTime = micros();
}

void goStraight(int pwm, int cycle_time)
{
  int left_pwm, right_pwm;
  float left_period = 0, right_period = 0;
  static unsigned long prev_leftCount = 0, prev_rightCount = 0;
  unsigned long diff_leftCount = 0, diff_rightCount = 0;

  diff_leftCount = leftCount - prev_leftCount;
  prev_leftCount = leftCount;

  diff_rightCount = rightCount - prev_rightCount;
  prev_rightCount = rightCount;

  if (diff_rightCount != 0 && diff_leftCount != 0)
  {
    left_period = cycle_time / (float) diff_leftCount;
    right_period = cycle_time / (float) diff_rightCount;
  }

  proportional_ = left_period - right_period;
  derivative_ = proportional_ - last_proportional_;
  integral_ += proportional_;
  last_proportional_ = proportional_;

  error_ = Kp_ * proportional_ + Kd_ * derivative_ + Ki_ * integral_;

  left_pwm = pwm + error_;
  right_pwm = pwm - error_;

  if (left_pwm > MAX_PWM)
    left_pwm = MAX_PWM;

  if (left_pwm < 0)
    left_pwm = 0;

  if (right_pwm > MAX_PWM)
    right_pwm = MAX_PWM;

  if (right_pwm < 0)
    right_pwm = 0;

  motor_left(left_pwm, 1);
  motor_right(right_pwm, 1);
}

void followWallFromBothSides(int pwm)
{
  int left_pwm, right_pwm;
  int left_dist, right_dist;

/*  left_dist = sonar[2].ping_cm();*/
  left_dist = 0;
  delay(30);
/*  right_dist = sonar[1].ping_cm();*/
  right_dist = 0;
  delay(30);

  _proportional = left_dist - right_dist;
  _derivative = _proportional - _last_proportional;
  _integral += _proportional;
  _last_proportional = _proportional;

  _error = _Kp * _proportional + _Kd * _derivative + _Ki * _integral;

  left_pwm = pwm - _error;
  right_pwm = pwm + _error;

  if (left_pwm > MAX_PWM)
    left_pwm = MAX_PWM;

  if (left_pwm < 0)
    left_pwm = 0;

  if (right_pwm > MAX_PWM)
    right_pwm = MAX_PWM;

  if (right_pwm < 0)
    right_pwm = 0;

  motor_left(left_pwm, 1);
  motor_right(right_pwm, 1);
}

void driveDistance(int cm_distance)
{
  rightCount=0;
  leftCount=0;
  int distanceToTravel = cm_distance;
  encoderTicks = ((float)1092/(float)(8.2*3.14)) * distanceToTravel ; //((float)(300)/(float)(4.2*3.14))
  while((encoderTicks >= leftCount) || (encoderTicks >= rightCount))
  {
    goStraight(100, 1);
    delay(1);
  }
  stopRobot();
  encoderTicks = 0;
  leftCount = 0;
  rightCount = 0;
  distanceToTravel = 0;
}

void driveDistance_withUltrasonic(int cm_distance)
{
  rightCount=0;
  leftCount=0;
  int distanceToTravel = cm_distance;
  encoderTicks = ((float)1092/(float)(8.2*3.14)) * distanceToTravel ; //((float)(300)/(float)(4.2*3.14))
  while((encoderTicks >= leftCount) || (encoderTicks >= rightCount))
  {
    followWallFromBothSides(100);
  }
  stopRobot();
  encoderTicks = 0;
  leftCount = 0;
  rightCount = 0;
  distanceToTravel = 0;
}
