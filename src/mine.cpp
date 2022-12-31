// Backup Plan for Mine cpp.
#include <ros/ros.h>
#include <iostream>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <math.h>

//macro for converting feet to meters 
#define FT_TO_M(wp) ((wp) * 0.3048)

// Declaring the AMCL pose for the mines
double poseAMCLx, poseAMCLy, poseAMCLz, poseAMCLa;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
        MoveBaseClient;

// Creating a goal to send to move_base using the move_base_msgs::MoveBaseGoal message type
move_base_msgs::MoveBaseGoal goal;

// Creating the array of waypoints for the mines
//float x[] = {4, 8.5, 8.5, 11, 11, 11, 8, 6, 11, 1, 1.1, 3, 3, 3, 11, 5, 1, 1, 3, 3, 13, 13, 11, 5, 5, 5, 1, 1};
//float y[] = {1, 1, 5, 5, 7, 11, 11, 11, 9, 9, 5.3, 5, 7, 9, 11, 11, 11, 13, 13, 11, 9, 13, 9, 9, 5, 3, 3, 1};

float x[] = {4, 8, 9, 11, 11, 11, 9, 6, 3, 1.1, 1.1, 2.7, 11, 3, 11, 5, 5, 5, 5, 3, 1, 13, 13, 13};
float y[] = {1, 1, 5, 5, 7, 11, 11, 11, 11, 10.8, 12.8, 12.8, 9, 7, 9, 9, 7, 5, 3, 3, 3, 9, 11, 12.8};
// Define the waypoints.
int wp = 0;

// define the total no of waypoints 
int total_wp = 24;

// Defining the state
int state;

// distance measured by the frony lidars 
int lidardata1, lidardata2;

//Creating call back functions for the lidar data, these will be called by spinOnce
void lidar1Callback(const std_msgs::Int16::ConstPtr& msg1)
{
	lidardata1 = msg1->data;
}

void lidar2Callback(const std_msgs::Int16::ConstPtr& msg2)
{
	lidardata2 = msg2->data;
}

//amcl callback function
void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL)
{
    poseAMCLx = msgAMCL->pose.pose.position.x;
    poseAMCLy = msgAMCL->pose.pose.position.y;
    poseAMCLa = msgAMCL->pose.pose.orientation.w;
    poseAMCLz = msgAMCL->pose.pose.orientation.z;
}

//The mine function is responsible to detect the mines it returns a 0(False) no mine found  or a 1(True) for a mine found
int mine()
{
    if((lidardata2<120) && ((lidardata1-lidardata2)>650))
    {
        ROS_INFO("Mine found");
        return 0;
    }
    else
    {
        return 1;
    }

}


float get_z_rotation()
{
	return std::atan2(poseAMCLa, poseAMCLz);
}

void get_pose()
{
	float theta;
	ros::spinOnce();
	theta = get_z_rotation();
	ROS_INFO("Mine found at Location: -- > %.2f <, > %.2f <, > %.2f<", poseAMCLx, poseAMCLy, theta);
	return;
}

void hunt()
{
	MoveBaseClient ac("move_base", true);
	int mine_return = 1;

	ROS_INFO("HUNT STATE");
	
	while(total_wp)
	{
		goal.target_pose.pose.position.x = x[wp]* 0.3048;
        	goal.target_pose.pose.position.y = y[wp]* 0.3048;
        	goal.target_pose.pose.orientation.z = 0;
        	goal.target_pose.pose.orientation.w = 1.0;
		ac.sendGoal(goal);
	
		while(1)
		{
			ros::spinOnce();
			mine_return = mine();
		
			if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && mine_return == 1)
			{
				ROS_INFO("Acheived waypoint <%d> ( %.2f, %.2f)", wp, x[wp], y[wp]);
				wp++;
				total_wp--;
				break;
			}
			else if(mine_return == 0)
			{
				wp++;
				total_wp--;
				state = 1;  // trash
				return;
			}
		}
	}
	ROS_INFO("DONE WITH TASK");
	return;
}

void trash()
{


	MoveBaseClient ac("move_base", true);

	//Velocity publisher setup
	ros::NodeHandle nv;
	ros::Publisher vel_pub = nv.advertise <geometry_msgs::Twist> ("/cmd_vel", 100);
	geometry_msgs::Twist vel_msg;

	//Servo motor setup
	ros::NodeHandle ns;
	ros::Publisher servo_pub = ns.advertise <std_msgs::Int16> ("/servo", 1);
	std_msgs::Int16 servo_msg;
	
	ros::spinOnce;

	ROS_INFO("Odom initiated to move forward");
	//now we will back up a little bit to not clog the mine in the robot
	vel_msg.linear.x = 0.1;
	vel_pub.publish(vel_msg);
	//half send delay should be okay
	ros::Duration(0.3).sleep();
	//now lets set the velocity back to 0
	vel_msg.linear.x = 0;
	vel_pub.publish(vel_msg);
	ROS_INFO("Odom terminated");

	//print location
	get_pose();
	
	servo_msg.data = 1;
	servo_pub.publish(servo_msg);

	ros::Duration(0.5).sleep();

	if(lidardata2 > 70)
	{// false reading -- open back the arms
		state = 0;
		servo_msg.data = 0;
		servo_pub.publish(servo_msg);
		state = 0;
		return;
	}



	// Go to trash 
	goal.target_pose.pose.position.x = FT_TO_M(11.5);
        goal.target_pose.pose.position.y = FT_TO_M(2);
        goal.target_pose.pose.orientation.z = 0;
        goal.target_pose.pose.orientation.w = 1.0;
        ROS_INFO("Sending goal to TRASH SITE");
	ac.sendGoal(goal);
	ROS_INFO("Goal sent to the robot to go to the Trash site");
	ac.waitForResult();
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Robot at trash site disposing the mine");
		
		//publish back to the arduino to open the arms
		servo_msg.data = 0;
		servo_pub.publish(servo_msg);

		ROS_INFO("Odom initiated to backup");

		
		//now we will back up a little bit to not clog the mine in the robot
		vel_msg.linear.x = -0.1;
		vel_pub.publish(vel_msg);
		//half send delay should be okay
		ros::Duration(3).sleep();
		//now lets set the velocity back to 0
		vel_msg.linear.x = 0;
		vel_pub.publish(vel_msg);
		ROS_INFO("Odom terminated");
		
		//returning to the refrence point
		goal.target_pose.pose.position.x = FT_TO_M(11);
        	goal.target_pose.pose.position.y = FT_TO_M(7);
        	goal.target_pose.pose.orientation.z = 0;
        	goal.target_pose.pose.orientation.w = 1.0;
        	ROS_INFO("Sending goal to get to the reference point");
		ac.waitForResult();
	}
	else 
	{
		ROS_INFO("Could not get to the Trash site");
		//open the arms and cmd_vel to back up
		servo_msg.data = 0;
		servo_pub.publish(servo_msg);

		ROS_INFO("Odom initiated to backup");
		//now we will back up a little bit to not clog the mine in the robot
		vel_msg.linear.x = -0.1;
		vel_pub.publish(vel_msg);
		//3 sec delay should be okay
		ros::Duration(3).sleep();
		//now lets set the velocity back to 0
		vel_msg.linear.x = 0;
		vel_pub.publish(vel_msg);
		ROS_INFO("Odom terminated");
	}	
	state = 0; //HUNT
	return;
}

void check_state()
{
   switch(state)
	{
		case 0:
		// HUNT
			hunt();
			break;
		case 1:
		// TRASH 
			trash();
			break;
	}
	return;
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "challenge_state_machines");
	MoveBaseClient ac("move_base", true);

	ros::NodeHandle n;
	ros::Subscriber lidar1_sub;
	ros::Subscriber lidar2_sub;

	//liadar subsciber setup
	lidar1_sub = n.subscribe("lidar1", 5, lidar1Callback);
	lidar2_sub = n.subscribe("lidar2", 5, lidar2Callback);

	ros::NodeHandle ns;
	ros::Publisher servo_pub = ns.advertise <std_msgs::Int16> ("/servo", 1);
	std_msgs::Int16 servo_msg;

	//publish back to the arduino to open the arms
	servo_msg.data = 0;
	servo_pub.publish(servo_msg);

	//amcl pose data 
	ros::NodeHandle nam;
	ros::Subscriber sub_amcl = nam.subscribe("amcl_pose", 1, poseAMCLCallback);
	
	while(!ac.waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Waiting for the move_base action server to come up");
	}
	
	//get all the call back functions
	ros::spinOnce();
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
	
	state = 0;
	wp = 0;

	while(1)
	{
	   // check state;
	   check_state();
	}
	
	ros::spin();
	return 0;
}

