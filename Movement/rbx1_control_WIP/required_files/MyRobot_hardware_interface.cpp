#include <YOUR_PACKAGE_NAME/MYRobot_hardware_interface.h>

​

MyRobot::MyRobot(ros::NodeHandle& nh) : nh_(nh) {

 

// Declare all JointHandles, JointInterfaces and JointLimitInterfaces of the robot.
    init();
    
// Create the controller manager
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    
//Set the frequency of the control loop.
    loop_hz_=10;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
    
//Run the control loop
    my_control_loop_ = nh_.createTimer(update_freq, &MyRobot::update, this);
}

​

MyRobot::~MyRobot() {
}

​

void MyRobot::init() {
        
// Create joint_state_interface for Joint1
    hardware_interface::JointStateHandle jointStateHandle1("joint_1_base_shoulder", &joint_position_[0], &joint_velocity_[0], &joint_effort_[0]);
    joint_state_interface_.registerHandle(jointStateHandle1);
// Create effort joint interface as Joint1 accepts effort command.
    hardware_interface::JointHandle jointEffortHandle1(jointStateHandle1, &joint_effort_command_[0]);
    effort_joint_interface_.registerHandle(jointEffortHandle1); 
// Create Joint Limit interface for Joint1
    joint_limits_interface::getJointLimits("joint_1_base_shoulder", nh_, limits);
    joint_limits_interface::EffortJointSaturationHandle jointLimitsHandle1(jointEffortHandle1, limits);
    effortJointSaturationInterface.registerHandle(jointLimitsHandle1);    


    
// Create joint_state_interface for Joint2
    hardware_interface::JointStateHandle jointStateHandle2("joint_2_shoulder_arm", &joint_position_[0], &joint_velocity_[0], &joint_effort_[0]);
    joint_state_interface_.registerHandle(jointStateHandle2);
// Create effort joint interface as Joint2 accepts effort command.
    hardware_interface::JointHandle jointEffortHandle2(jointStateHandle2, &joint_effort_command_[0]);
    effort_joint_interface_.registerHandle(jointEffortHandle2); 
// Create Joint Limit interface for Joint2
    joint_limits_interface::getJointLimits("joint_2_shoulder_arm", nh_, limits);
    joint_limits_interface::EffortJointSaturationHandle jointLimitsHandle2(jointEffortHandle2, limits);
    effortJointSaturationInterface.registerHandle(jointLimitsHandle2);



// Create joint_state_interface for Joint3
    hardware_interface::JointStateHandle jointStateHandle3("joint_3_arm_upper_forearm", &joint_position_[0], &joint_velocity_[0], &joint_effort_[0]);
    joint_state_interface_.registerHandle(jointStateHandle3);
// Create effort joint interface as Joint3 accepts effort command.
    hardware_interface::JointHandle jointEffortHandle3(jointStateHandle3, &joint_effort_command_[0]);
    effort_joint_interface_.registerHandle(jointEffortHandle3); 
// Create Joint Limit interface for Joint3
    joint_limits_interface::getJointLimits("joint_3_arm_upper_forearm", nh_, limits);
    joint_limits_interface::EffortJointSaturationHandle jointLimitsHandle3(jointEffortHandle3, limits);
    effortJointSaturationInterface.registerHandle(jointLimitsHandle3);



// Create joint_state_interface for Joint4
    hardware_interface::JointStateHandle jointStateHandle4("joint_4_upper_forearm_forearm", &joint_position_[0], &joint_velocity_[0], &joint_effort_[0]);
    joint_state_interface_.registerHandle(jointStateHandle4);
// Create effort joint interface as Joint4 accepts effort command.
    hardware_interface::JointHandle jointEffortHandle4(jointStateHandle4, &joint_effort_command_[0]);
    effort_joint_interface_.registerHandle(jointEffortHandle4); 
// Create Joint Limit interface for Joint4
    joint_limits_interface::getJointLimits("joint_4_upper_forearm_forearm", nh_, limits);
    joint_limits_interface::EffortJointSaturationHandle jointLimitsHandle4(jointEffortHandle4, limits);
    effortJointSaturationInterface.registerHandle(jointLimitsHandle4);



// Create joint_state_interface for Joint5
    hardware_interface::JointStateHandle jointStateHandle5("joint_5_forearm_wrist", &joint_position_[0], &joint_velocity_[0], &joint_effort_[0]);
    joint_state_interface_.registerHandle(jointStateHandle5);
// Create effort joint interface as Joint5 accepts effort command.
    hardware_interface::JointHandle jointEffortHandle5(jointStateHandle5, &joint_effort_command_[0]);
    effort_joint_interface_.registerHandle(jointEffortHandle5); 
// Create Joint Limit interface for Joint5
    joint_limits_interface::getJointLimits("joint_5_forearm_wrist", nh_, limits);
    joint_limits_interface::EffortJointSaturationHandle jointLimitsHandle5(jointEffortHandle5, limits);
    effortJointSaturationInterface.registerHandle(jointLimitsHandle5);



// Create joint_state_interface for Joint6
    hardware_interface::JointStateHandle jointStateHandle6("joint_6_wrist_hand", &joint_position_[0], &joint_velocity_[0], &joint_effort_[0]);
    joint_state_interface_.registerHandle(jointStateHandle6);
// Create effort joint interface as Joint6 accepts effort command.
    hardware_interface::JointHandle jointEffortHandle6(jointStateHandle6, &joint_effort_command_[0]);
    effort_joint_interface_.registerHandle(jointEffortHandle6); 
// Create Joint Limit interface for Joint6
    joint_limits_interface::getJointLimits("joint_6_wrist_hand", nh_, limits);
    joint_limits_interface::EffortJointSaturationHandle jointLimitsHandle6(jointEffortHandle6, limits);
    effortJointSaturationInterface.registerHandle(jointLimitsHandle6);



// Register all joints interfaces    
    registerInterface(&joint_state_interface_);
    registerInterface(&effort_joint_interface_);
    registerInterface(&effortJointSaturationInterface);   
}

​

//This is the control loop
void MyRobot::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

​

void MyRobot::read() {​

  // Write the protocol (I2C/CAN/ros_serial/ros_industrial)used to get the current joint position and/or velocity and/or effort       

  //from robot.
  // and fill JointStateHandle variables joint_position_[i], joint_velocity_[i] and joint_effort_[i] for Joint i

}

​

void MyRobot::write(ros::Duration elapsed_time) {
  // Safety
  effortJointSaturationInterface.enforceLimits(elapsed_time);   // enforce limits for joints


  // Write the protocol (I2C/CAN/ros_serial/ros_industrial)used to send the commands to the robot's actuators.
  // the output commands need to send are joint_effort_command_[i] for Joint i

}

​

int main(int argc, char** argv)
{

    //Initialze the ROS node.
    ros::init(argc, argv, "MyRobot_hardware_interface_node");
    ros::NodeHandle nh;
    
    //Separate Sinner thread for the Non-Real time callbacks such as service callbacks to load controllers
    ros::MultiThreadedspinner(2); 
    
    
    // Create the object of the robot hardware_interface class and spin the thread. 
    MyRobot ROBOT(nh);
    spinner.spin();
    
    return 0;
}
