#include <Arduino.h>
#include <ros.h>
#include "arduino_robot/Joystick.h"

#define X_AXIS 6
#define Y_AXIS 7
#define BUTTON 12
#define DELAY 200

int button_state = 0;

arduino_robot::Joystick jdata;

ros::Publisher pub_jdata("/arduino_robot/joystick", &jdata);
ros::NodeHandle  nh;

void setup()
{
    nh.initNode();
    nh.advertise(pub_jdata);  

    pinMode(X_AXIS, INPUT);
    pinMode(Y_AXIS, INPUT);

    // Activate pull-up resistor for button
    pinMode(BUTTON, INPUT_PULLUP);
}

unsigned long publisher_timer;

void loop()
{
    if (millis() > publisher_timer) {

        jdata.x = analogRead(X_AXIS);
        jdata.y = analogRead(Y_AXIS);

        jdata.button = digitalRead(BUTTON) ? 0 : 1;

        delay(75);

        publisher_timer = millis() + DELAY;
        pub_jdata.publish(&jdata);
    }

    nh.spinOnce();
}
