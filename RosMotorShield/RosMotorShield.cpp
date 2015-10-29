#include <ros.h>
#include <AFMotor.h>
#include "arduino_robot/MotorSpeed.h"
#include "arduino_robot/ArmMotors.h"

AF_DCMotor m1(1, MOTOR12_64KHZ);
AF_DCMotor m2(2, MOTOR12_64KHZ);

int state = 0;
int speed[] = {0, 0};

ros::NodeHandle  nh;

void setSpeed(int newSpeedM1, int newSpeedM2)
{
    // We get '0-1023' and motor speed is '0-255' + direction
    int m1s = (newSpeedM1 - 512) / 2;
    int m2s = (newSpeedM2 - 512) / 2;

    // Check values
    if (m1s < -255)
        m1s = -255;
    if (m1s > 255)
        m1s = 255;

    if (m2s < -255)
        m2s = -255;
    if (m2s > 255)
        m2s = 255;

    // Set direction
    if (m1s < 0)
        m1.run(BACKWARD);
    else
        m1.run(FORWARD);

    if (m2s < 0)
        m2.run(BACKWARD);
    else
        m2.run(FORWARD);

    // Accelerate
    m1.setSpeed(newSpeedM1);  
    m2.setSpeed(newSpeedM2);  

    delay(50);
}

// Callback to arm and disarm
void arm_cb(const arduino_robot::ArmMotors & msg)
{
    // Change state if button clicked
    state = msg.state ? 1 : 0;

    // Handle state
    if (!state) {
        speed[0] = 0;
        m1.setSpeed(speed[0]);  
        speed[1] = 0;
        m2.setSpeed(speed[1]);  

        delay(50);
    }
}

// Callback to set motor speed
void motors_cb(const arduino_robot::MotorSpeed & msg)
{
    if (state) {
        if (msg.m1 != speed[0]) {
            speed[0] = msg.m1;
            m1.setSpeed(speed[0]);  
        }

        if (msg.m2 != speed[1]) {
            speed[1] = msg.m2;
            m2.setSpeed(speed[1]);  
        }
    }

    delay(50);
}

// Subscribe to topics
ros::Subscriber<arduino_robot::ArmMotors> 
sub_arm("/arduino_robot/arm_motors", arm_cb);

ros::Subscriber<arduino_robot::MotorSpeed> 
sub_speed("/arduino_robot/motor_speed", motors_cb);

void setup()
{
    m1.run(RELEASE);
    m2.run(RELEASE);

    m1.run(FORWARD);
    m2.run(FORWARD);

    nh.initNode();

    // Subscribe to topics
    nh.subscribe(sub_arm);
    nh.subscribe(sub_speed);

}

void loop()
{
    nh.spinOnce();
}
