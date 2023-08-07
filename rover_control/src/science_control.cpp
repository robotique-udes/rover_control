#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <cmath>

sensor_msgs::JointState joint_state;

bool ARRIVED = false;
double PI = 3.14159265358979323846;
double closest_multiple_2pi;

/* MOTOR SPEED */
float SPEED = 1.0;
float NEG_SPEED = -1.0;

/* SET CLOSED POSITION */
float CLOSED_POS = 260.36; //Position 0 in rad of turntable

/* SET REST POSITION */
float REST_POS = -1.23025; //Rest position of palm in rad

ros::Publisher pub;

bool first_call = true;  // Flag to track the first call to reach_slot

void cb_joint_states(const sensor_msgs::JointState msg) {
    joint_state = msg;
}

void multiple() {
    closest_multiple_2pi = 2 * PI * std::round((joint_state.position[0] - CLOSED_POS) / (2 * PI));

    ROS_WARN_STREAM("closest_multiple_2pi: " << closest_multiple_2pi);

    if (closest_multiple_2pi == 0) {
        closest_multiple_2pi = 1;
    }
}

void reach_slot(int slot_number) {
    if (ARRIVED) {
        return;  // Return if the goal is already reached
    }

    sensor_msgs::JointState msg;
    msg.name.push_back("science_rotation");

    // Check if it's the first call to reach_slot in a sequence
    if (first_call) {
        multiple();  // Call multiple() only for the first time
        first_call = false;  // Set the flag to false for subsequent calls
    }

    double desired_position = closest_multiple_2pi + CLOSED_POS + (57 * PI / 180.0 * slot_number);

    ROS_WARN_STREAM("Desired position: " << desired_position);

    double position_error = desired_position - joint_state.position[0];

    ROS_WARN_STREAM("Position error: " << position_error);

    // Determine the direction of movement based on the sign of position_error
    float direction = (position_error < 0) ? NEG_SPEED : SPEED;

    // If the current joint state position is close to the desired position, stop the motor
    if (std::abs(position_error) < 0.05) {
        ARRIVED = true;  // Set the ARRIVED flag to true
        msg.velocity.push_back(0.0);  // Stop the motor
    } else {
        // Move the motor towards the desired position using the determined direction
        msg.velocity.push_back(direction);
    }

    pub.publish(msg);
}


void home () {

    if (ARRIVED) {
        return;  // Return if the goal is already reached
    }

    sensor_msgs::JointState msg;
    msg.name.push_back("science_shovel");
    
    double desired_position = REST_POS;

    ROS_WARN_STREAM("Desired position: " << desired_position);

    double position_error = desired_position - joint_state.position[0];

    ROS_WARN_STREAM("Position error: " << position_error);

    // Determine the direction of movement based on the sign of position_error
    float direction = (position_error < 0) ? NEG_SPEED : SPEED;

  // If the current joint state position is close to the desired position, stop the motor
    if (std::abs(position_error) < 0.05) {
        ARRIVED = true;  // Set the ARRIVED flag to true
        msg.velocity.push_back(0.0);  // Stop the motor
    } else {
        // Move the motor towards the desired position using the determined direction
        msg.velocity.push_back(direction);
    }

    pub.publish(msg);
}

void swing () {

     if (ARRIVED) {
        return;  // Return if the goal is already reached
    }

      sensor_msgs::JointState msg;
    msg.name.push_back("science_shovel");
    
    double desired_position = REST_POS + (95*PI/180.0);

    ROS_WARN_STREAM("Desired position: " << desired_position);

    double position_error = desired_position - joint_state.position[0];

    ROS_WARN_STREAM("Position error: " << position_error);

     // Determine the direction of movement based on the sign of position_error
    float direction = (position_error < 0) ? NEG_SPEED : SPEED;

   // If the current joint state position is close to the desired position, stop the motor
    if (std::abs(position_error) < 0.05) {
        ARRIVED = true;  // Set the ARRIVED flag to true
        msg.velocity.push_back(0.0);  // Stop the motor
    } else {
        // Move the motor towards the desired position using the determined direction
        msg.velocity.push_back(direction);
    }

    pub.publish(msg);

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dynamixel_controller_node");

    ros::NodeHandle nh;
    pub = nh.advertise<sensor_msgs::JointState>("desired_joint_states", 1);

    ros::Subscriber sub = nh.subscribe("joint_states", 1, cb_joint_states);

    ros::Rate loop_rate(10); // Adjust the loop rate as needed

    // Wait until joint_state is initialized with valid data
    while (ros::ok() && joint_state.position.empty()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    while (ros::ok()) {
        if (!joint_state.position.empty()) {
          
            reach_slot(0);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

