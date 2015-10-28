#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <AFMotor.h>

AF_DCMotor m1(1);
AF_DCMotor m2(2);

ros::NodeHandle  nh;

// Callback function to set motor speed
void motors_cb(const std_msgs::Int16MultiArray & motor_speed)
{
    int m1s = (motor_speed.data[0] - 512)/2;
    int m2s = (motor_speed.data[1] - 512)/2;

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
    m1.setSpeed(m1s);  
    m2.setSpeed(m2s);  

    delay(100);
}

ros::Subscriber<std_msgs::Int16MultiArray> 
    sub_speed("/arduino_robot/motor_speed", motors_cb);

void setup()
{
  m1.run(RELEASE);
  m2.run(RELEASE);

  nh.initNode();

  // Subscribe to topic 'arduino_robot/motor_speed'
  nh.subscribe(sub_speed);

}

void loop()
{
  
  nh.spinOnce();
}
