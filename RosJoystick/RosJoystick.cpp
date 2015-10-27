#include <Arduino.h>
#include "ros.h"
#include "std_msgs/UInt16MultiArray.h"

#define X_AXIS 6
#define Y_AXIS 7
#define BUTTON 13

#define DELAY 200

unsigned int button = 0;

std_msgs::UInt16MultiArray jdata;
ros::Publisher pub_jdata("joystick", &jdata);
ros::NodeHandle  nh;

void setup()
{
  nh.initNode();
  jdata.data_length = 2;
  nh.advertise(pub_jdata);  
  
  pinMode(X_AXIS, INPUT);
  pinMode(Y_AXIS, INPUT);
  pinMode(BUTTON, INPUT);
  digitalWrite(BUTTON, HIGH);
}

long publisher_timer;

void loop()
{
  if (millis() > publisher_timer) {

    jdata.data[0] = analogRead(X_AXIS);
    jdata.data[1] = analogRead(Y_AXIS);
    delay(50);

    publisher_timer = millis() + DELAY;
    pub_jdata.publish(&jdata);
  }
  
  nh.spinOnce();
}
