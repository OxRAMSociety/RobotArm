#include <ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19
//#define stepSize 1
//#define gearRatio 1

#define DEGREES_STEP 0.1125

ros::NodeHandle  nh;

float desiredAngle = 0;
float currentAngle = 0;
float diffAngle = 0;

std_msgs::Float64 mydata;

void messageCb(const sensor_msgs::JointState& cmd_msg)
{
desiredAngle = cmd_msg.position[2]*360/(2*3.1419);
//Serial1.println(desiredAngle);
  
}

ros::Subscriber<sensor_msgs::JointState> sub("/move_group/fake_controller_joint_states", messageCb );

void setup()
{ 
 // Serial1.begin(9600);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  pinMode(Z_STEP_PIN, OUTPUT);
  pinMode(Z_DIR_PIN, OUTPUT);
  pinMode(Z_ENABLE_PIN, OUTPUT);

  digitalWrite(Z_ENABLE_PIN, LOW);
 // pinMode(8, OUTPUT);
 // digitalWrite(8,HIGH);
 // Serial1.println("Testing");
 // delay(5000);
 // digitalWrite(8,LOW);
}

void loop()
{  
  //mydata.data = desiredAngle;
  //Serial1.println(desiredAngle);
  nh.spinOnce();

  diffAngle = desiredAngle - currentAngle;

  if (diffAngle > 0) {
    digitalWrite(Z_DIR_PIN, HIGH);
  } else { 
    digitalWrite(Z_DIR_PIN, LOW);
  }
  
  if (abs(diffAngle) >= 10*DEGREES_STEP) {
    digitalWrite(Z_STEP_PIN, HIGH);
    delay(1);
    digitalWrite(Z_STEP_PIN, LOW);
    currentAngle += DEGREES_STEP;
  }
}
