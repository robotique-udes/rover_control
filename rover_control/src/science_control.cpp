#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

double PI = 3.14159265358979323846;
double DEG2RAD = PI/180.0;

sensor_msgs::JointState joint_state;
ros::Publisher pub;

bool ARRIVED = false;
float ERR_FACTOR = 0.05;

/* SET MOTOR SPEED */
float SPEED = 1.0;
float NEG_SPEED = -SPEED;

/* SET CLOSED POSITION OF TURNTABLE (> 2PI) */
float CLOSED_POS = 260.36; 

/* SET REST POSITION OF PALM (FURTHEST BACK POSITION >0) */
float REST_POS = 1.525; 
 
void cb_joint_states(const sensor_msgs::JointState msg) {joint_state = msg;}
double get_joint_state(){return joint_state.position[0];}

/* 
    EITHER REACH ONE OF THE TURNTABLE SLOTS OR SWING THE PALM
    - slot_number IS ANYTHING BETWEEN 0 AND 5 BUT MUST BE DEFINED FOR TURNTABLE
*/
void reach_goal(int slot_number = 6) {

    if (ARRIVED) {return;} // Return if the goal is already reached
    
    bool shovel = 0;
    double slot = static_cast<double>(slot_number);
    double desired_position;
    double position_error;
    double direction;
    double current_position = get_joint_state();
    
    if (slot_number == 6){shovel = 1; slot = 1.0;}

    sensor_msgs::JointState msg;

    if (shovel){msg.name.push_back("science_shovel");}
    else {msg.name.push_back("science_rotation");}

    if (!shovel) {desired_position = CLOSED_POS + (60.0 * DEG2RAD * slot);}
    else {

        if (REST_POS - ERR_FACTOR < current_position < REST_POS + ERR_FACTOR){desired_position = REST_POS + (95.0 * DEG2RAD * slot);}
        else {desired_position = REST_POS;}

        }

    position_error = desired_position - current_position;

    direction = (position_error < 0) ? NEG_SPEED : SPEED;

    if (std::abs(position_error) < ERR_FACTOR) {

        ARRIVED = true;  
        msg.velocity.push_back(0.0);  // Stop the motor

    } else {msg.velocity.push_back(direction);}

    pub.publish(msg);
    
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "dynamixel_controller_node");

    ros::NodeHandle nh;
    pub = nh.advertise<sensor_msgs::JointState>("desired_joint_states", 1);

    ros::Subscriber sub = nh.subscribe("joint_states", 1, cb_joint_states);

    ros::Rate loop_rate(10); 

    while (ros::ok() && joint_state.position.empty()) {

        ros::spinOnce();
        loop_rate.sleep();
    }

    while (ros::ok()) {

        if (!joint_state.position.empty()) {reach_goal(0);}

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

