/*
 * The receiver node for the arms joint control.
 *
 * Takes in the information from the arm_relay.py
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <math.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>


// Shoulder Joint
int b1 = 2;
int b2 = 3;

// Elbow joint
int s1 = 7;
int s2 = 8;

// Wrist Joint
int w1 = 11;
int w2 = 12;

int pwm1 = 4;
int pwm2 = 9;
int pwm3 = 13;

ros::NodeHandle  nh;
long int prev_command_time = millis();

void forward(int pin1, int pin2)
{
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, HIGH);
}


void backward(int pin1, int pin2)
{
  digitalWrite(pin1, HIGH);
  digitalWrite(pin2, LOW);
}

void stop(int pin1, int pin2)
{
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, LOW);
}

void decay(long int curr_time)
{
  int diff_time = curr_time - prev_command_time;
  if (diff_time < 500){
    float percent_time = 1 - diff_time / 500;
    int pwm_signal = (int) percent_time * 255;
    analogWrite(pwm1, pwm_signal);
    analogWrite(pwm2, pwm_signal);
    analogWrite(pwm3, pwm_signal);
  }
  else{
     stop(b1, b2);
     stop(s1, s2);
     stop(w1, w2);
  }
}

void check_value(int check, int pin1, int pin2)
{
  if (check > 1)
  {
    forward(pin1, pin2);
  }
  else if (check < -1)
  {
    backward(pin1, pin2);
  }
  else{
    stop(pin1, pin2);
  }
 
}

void relay_cb( const std_msgs::Int16MultiArray& cmd_msg){
  int base = cmd_msg.data[0];
  int shoulder = cmd_msg.data[1];
  int wrist = cmd_msg.data[2];
  check_value(base, b1, b2);
  check_value(shoulder, s1, s2);
  check_value(wrist, w1, w2);
   analogWrite(pwm1, 255);
   analogWrite(pwm2, 255);
   analogWrite(pwm3, 255);
  prev_command_time = millis();
}


ros::Subscriber<std_msgs::Int16MultiArray> sub("arm_joints", relay_cb);

void setup(){
  pinMode(b1, OUTPUT);
  pinMode(b2, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(w1, OUTPUT);
  pinMode(w2, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop(){
  long int curr_time = millis();
  decay(curr_time);
  nh.spinOnce();
  delay(1);
}
