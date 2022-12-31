//include ROS Libraries
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

// Declaring the AMCL pose for the mines.
double poseAMCLx, poseAMCLy, poseAMCLa, poseAMCLz;

//initial state will be set to HUNT to go to waypoints to find known waypoints
int state = 0;
int trash_hit = 0;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
        MoveBaseClient;


// Creating a goal to send to move_base using the move_base_msgs::MoveBaseGoal message type
move_base_msgs::MoveBaseGoal goal;


// distance measured by the frony lidars 
int lidardata1, lidardata2;

//amcl callback function
void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL)
{
    poseAMCLx = msgAMCL->pose.pose.position.x;
    poseAMCLy = msgAMCL->pose.pose.position.y;
    poseAMCLa = msgAMCL->pose.pose.orientation.w;
    poseAMCLz = msgAMCL->pose.pose.orientation.z;
}

//creating arrays for the waypoints to go to known waypoints
float x[] = {4, 8.5, 8.5, 11, 11, 11, 8, 6.5, 11, 1, 1.1, 3, 3};
float y[] = {1, 1, 5, 5, 7, 11, 11, 11, 9, 9, 5.3, 5, 7};
int waypoint = 0;


//creating a variable to store number of waypoints
int number_of_state_hunt_waypoints = 13;


//Creating call back functions for the lidar data, these will be called by spinOnce
void lidar1Callback(const std_msgs::Int16::ConstPtr& msg1)
{
	lidardata1 = msg1->data;
}

void lidar2Callback(const std_msgs::Int16::ConstPtr& msg2)
{
	lidardata2 = msg2->data;
}

//The mine function is responsible to detect the mines it returns a 0(False) no mine found  or a 1(True) for a mine found
int mine()
{
    if((lidardata2<160) && ((lidardata1-lidardata2)>650))
    {
        ROS_INFO("Mine found");
        return 0;
    }
    else
    {
        //ROS_INFO("lidardata1: %d lidardata2: %d\n", lidardata1, lidardata2);
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

//Trash function is called once a mine is found its sets up a waypoint to the trash site 

void trash_state()
{

	ROS_INFO("Trash state");
	MoveBaseClient ac("move_base", true);

	//Velocity publisher setup
	ros::NodeHandle nv;
	ros::Publisher vel_pub = nv.advertise <geometry_msgs::Twist> ("/cmd_vel", 100);
	geometry_msgs::Twist vel_msg;

	ros::NodeHandle ns;
	ros::Publisher servo_pub = ns.advertise <std_msgs::Int16> ("/servo", 100);
	std_msgs::Int16 servo_msg;
	
	//set up a waypoint to the trash site and once the trash site waypoint is hit we need to go back the refrence point, increament the trash_hit counter and then return.

	//perform check if the arm has the mine

	ros::Duration(0.5).sleep();
	if(lidardata2 > 45)
	{
		state = 0;
		servo_msg.data = 0;
		servo_pub.publish(servo_msg);
		return;
	}

	//changes the wp when mine is found
        goal.target_pose.pose.position.x = FT_TO_M(11.5);
        goal.target_pose.pose.position.y = FT_TO_M(2);
        goal.target_pose.pose.orientation.z = 0;
        goal.target_pose.pose.orientation.w = 1.0;
        ROS_INFO("Sending goal to TRASH SITE");
	ac.sendGoal(goal);
	ROS_INFO("Goal sent to the robot to go to the Trash site");
	//while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){}
	ac.waitForResult();
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Robot at trash site disposing the mine");
		//trash_hit++;

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

		//while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){}

		ac.waitForResult();
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("Robot is at the reference point");
			//we want to set the state back to hunt
		}
		else
		{
			ROS_INFO("Could not get to the refernce point");
		}
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

	//set state back to HUNT
	state = 0;//HUNT
	return;
}

void hunt_state()
{

ROS_INFO("hunt state");
/*
float x[] = {4, 8, 8, 11, 11, 8, 6.5, 11, 1, 1, 3, 3};
float y[] = {1, 1, 5, 5.5, 11, 11, 11, 9, 9, 5, 5, 7};
int waypoint = 0; 
*/
	MoveBaseClient ac("move_base", true);
	int mine_return = 1;

	while(number_of_state_hunt_waypoints)
	{
		goal.target_pose.pose.position.x = x[waypoint]* 0.3048;
        	goal.target_pose.pose.position.y = y[waypoint]* 0.3048;
        	goal.target_pose.pose.orientation.z = 0;
        	goal.target_pose.pose.orientation.w = 1.0;
		ac.sendGoal(goal);
		//ROS_INFO("Going to waypoint <%d> ( %.2f, %.2f)", waypoint, x[waypoint], y[waypoint]);
		
		while(1)
		{
			ros::spinOnce();
			mine_return = mine();
			
			//check if the waypoint is achieved and keep checking if no mine is found then we go to the next waypoint
			if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && mine_return == 1)
			{
				ROS_INFO("Acheived waypoint <%d> ( %.2f, %.2f)", waypoint, x[waypoint], y[waypoint]);
				waypoint++;
				number_of_state_hunt_waypoints--;
				break;
			}
			//mine has been found, here we will change the state to trash and return however after the integration of arms we will set the state to armed
			else if(mine_return == 0)
			{
				waypoint++;
				number_of_state_hunt_waypoints--;

				//set the state to armed
				state = 1;
				return;
			}
		}
	}
	//set state to search the map for the unknown mines
	state = 4;
	return;
}

void armed_state()
{

	ROS_INFO("armed state");
	//Velocity publisher setup
	ros::NodeHandle nv;
	ros::Publisher vel_pub = nv.advertise <geometry_msgs::Twist> ("/cmd_vel", 100);
	geometry_msgs::Twist vel_msg;

	//Servo motor setup
	ros::NodeHandle ns;
	ros::Publisher servo_pub = ns.advertise <std_msgs::Int16> ("/servo", 1);
	std_msgs::Int16 servo_msg;

	ros::spinOnce;

	//zone in on the mine either using the lower data or proximity sensors(do not work in the sun beware!)

	int i = waypoint - 1;
	//ROS_INFO("Mine Found at x = %.2f  y = %.2f", x[i], y[i]);
	get_pose();

	ROS_INFO("Odom initiated to move forward");
	//now we will back up a little bit to not clog the mine in the robot
	vel_msg.linear.x = 0.1;
	vel_pub.publish(vel_msg);
	//half send delay should be okay
	ros::Duration(0.1).sleep();
	//now lets set the velocity back to 0
	vel_msg.linear.x = 0;
	vel_pub.publish(vel_msg);
	ROS_INFO("Odom terminated");

	//publish back to the arduino to close the arms
	servo_msg.data = 1;
	servo_pub.publish(servo_msg);
	state = 2;
	return;	
}

void unknown_mine_1()
{

	ROS_INFO("unknown mine 1 state");
	int n = 8;
	int mine_return;
	int i = 0;
	//gets to this waypoint turns to odom
	float x1[] = {11, 4, 3, 1.1, 1.1, 2.7, 3, 5};
	float y1[] = {11, 11, 11, 10.8, 12.8, 11, 11};
	//Velocity publisher setup
	ros::NodeHandle nv;
	ros::Publisher vel_pub = nv.advertise <geometry_msgs::Twist> ("/cmd_vel", 1);
	geometry_msgs::Twist vel_msg;

	//Servo motor setup
	
	ros::NodeHandle ns;
	ros::Publisher servo_pub = ns.advertise <std_msgs::Int16> ("/servo", 1);
	std_msgs::Int16 servo_msg;

	//create a path to drop mines in trash site and switch back to the search state
	MoveBaseClient ac("move_base", true);
	
	while(n != 0)
	{
		goal.target_pose.pose.position.x = x[i]* 0.3048;
        	goal.target_pose.pose.position.y = y[i]* 0.3048;
        	goal.target_pose.pose.orientation.z = 0;
        	goal.target_pose.pose.orientation.w = 1.0;
		ac.sendGoal(goal);
		ROS_INFO("Going to waypoint <%d> ( %.2f, %.2f)", waypoint, x[waypoint], y[waypoint]);
		
		while(1)
		{
			ros::spinOnce();
			mine_return = mine();
			
			//check if the waypoint is achieved and keep checking if no mine is found then we go to the next waypoint
			if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && mine_return == 1)
			{
				ROS_INFO("Acheived waypoint <%d> ( %.2f, %.2f)", waypoint, x[waypoint], y[waypoint]);
				i++;
				n--;
				break;
			}
			//mine has been found, here we will change the state to trash and return however after the integration of arms we will set the state to armed
			else if(mine_return == 0)
			{
				get_pose();				
				i++;
				n--;
				//set the state to armed
				state = 5;
				return;
			}
		}
	}
	state = 6;
	return;
}

void unknwown_trash1()
{

	ROS_INFO("unknown trash 1");
	//gets to this waypoint turns to odom
	float xt[] = {11, 12};
	float yt[] = {11, 2};
	//Velocity publisher setup
	ros::NodeHandle nv;
	ros::Publisher vel_pub = nv.advertise <geometry_msgs::Twist> ("/cmd_vel", 1);
	geometry_msgs::Twist vel_msg;

	//Servo motor setup
	ros::NodeHandle ns;
	ros::Publisher servo_pub = ns.advertise <std_msgs::Int16> ("/servo", 1);
	std_msgs::Int16 servo_msg;

	//create a path to drop mines in trash site and switch back to the search state
	MoveBaseClient ac("move_base", true);
	
	for(int b = 0; b<2; b++)	
	{
		goal.target_pose.pose.position.x = FT_TO_M(xt[b]);
        	goal.target_pose.pose.position.y = FT_TO_M(yt[b]);
        	goal.target_pose.pose.orientation.z = 0;
        	goal.target_pose.pose.orientation.w = 1.0;
        	ROS_INFO("Sending goal to waypoint unknown mine 1");
		ac.sendGoal(goal);
		ac.waitForResult();
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_INFO("going to trash");
		}
		else
		{
			ROS_INFO("Could not get to waypoint for trash unknown mine 1");
		}
	}
	
	//open my claws

	//perform backward odom
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

	//set goal to reference point
	goal.target_pose.pose.position.x = FT_TO_M(11);
        goal.target_pose.pose.position.y = FT_TO_M(7);
        goal.target_pose.pose.orientation.z = 0;
        goal.target_pose.pose.orientation.w = 1.0;
        ROS_INFO("Sending goal to get to the reference point");
	ac.sendGoal(goal);
	ac.waitForResult();
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Robot is at the reference point");
		//we want to set the state back to hunt
	}
	else
	{
		ROS_INFO("Could not get to the refernce point");
	}
	
	state = 6;
	return;
}

void unknown_mine_2()
{
	ROS_INFO("unknown mine 2");
	//gets to this waypoint turns to odom
	float x2[] = {13, 13, 13, 13};
	float y2[] = {7, 9, 10, 12.8};
	int mine_return;
	int i = 0;
	int n = 4;
	//Velocity publisher setup
	ros::NodeHandle nv;
	ros::Publisher vel_pub = nv.advertise <geometry_msgs::Twist> ("/cmd_vel", 1);
	geometry_msgs::Twist vel_msg;

	//Servo motor setup
	ros::NodeHandle ns;
	ros::Publisher servo_pub = ns.advertise <std_msgs::Int16> ("/servo", 1);
	std_msgs::Int16 servo_msg;

	//create a path to drop mines in trash site and switch back to the search state
	MoveBaseClient ac("move_base", true);
	
	while(n != 0)
	{
		goal.target_pose.pose.position.x = x[i]* 0.3048;
        	goal.target_pose.pose.position.y = y[i]* 0.3048;
        	goal.target_pose.pose.orientation.z = 0;
        	goal.target_pose.pose.orientation.w = 1.0;
		ac.sendGoal(goal);
		ROS_INFO("Going to waypoint <%d> ( %.2f, %.2f)", waypoint, x[waypoint], y[waypoint]);
		
		while(1)
		{
			ros::spinOnce();
			mine_return = mine();
			
			//check if the waypoint is achieved and keep checking if no mine is found then we go to the next waypoint
			if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && mine_return == 1)
			{
				ROS_INFO("Acheived waypoint <%d> ( %.2f, %.2f)", waypoint, x[waypoint], y[waypoint]);
				i++;
				n--;
				break;
			}
			//mine has been found, here we will change the state to trash and return however after the integration of arms we will set the state to armed
			else if(mine_return == 0)
			{
				get_pose();
				i++;
				n--;
				//set the state to armed
				state = 7;
				return;
			}
		}
	}
	state = 8;
	return;
}

void unknwown_trash2()
{

	ROS_INFO("unknown trash 2");
	//Velocity publisher setup
	ros::NodeHandle nv;
	ros::Publisher vel_pub = nv.advertise <geometry_msgs::Twist> ("/cmd_vel", 1);
	geometry_msgs::Twist vel_msg;

	//Servo motor setup
	
	ros::NodeHandle ns;
	ros::Publisher servo_pub = ns.advertise <std_msgs::Int16> ("/servo", 1);
	std_msgs::Int16 servo_msg;

	//create a path to drop mines in trash site and switch back to the search state
	MoveBaseClient ac("move_base", true);
	goal.target_pose.pose.position.x = FT_TO_M(12);
        goal.target_pose.pose.position.y = FT_TO_M(2);
        goal.target_pose.pose.orientation.z = 0;
       	goal.target_pose.pose.orientation.w = 1.0;
        ROS_INFO("Going to trash site");
	//while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){}
	ac.sendGoal(goal);
	ac.waitForResult();
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Got to trash site");
	}
	else
	{
		ROS_INFO("Could not get to trash site");
	}
		
	//open my claws
	//perform backward odom
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

	//set goal to reference point
	goal.target_pose.pose.position.x = FT_TO_M(11);
        goal.target_pose.pose.position.y = FT_TO_M(9);
        goal.target_pose.pose.orientation.z = 0;
        goal.target_pose.pose.orientation.w = 1.0;
        ROS_INFO("Sending goal to get to the reference point");
	ac.sendGoal(goal);
	//while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){}
	ac.waitForResult();
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Robot is at the reference point");
		//we want to set the state back to hunt
	}
	else
	{
		ROS_INFO("Could not get to the refernce point");
	}
	
	state = 8;
	return;
}

void unknown_mine_3()
{

	ROS_INFO("unknown mine 3");
	float x3[] = {5, 5, 5, 5, 1};
	float y3[] = {9, 7, 5, 3, 3};
	
	int c = 5;
	int q = 0;
	
	MoveBaseClient ac("move_base", true);
	int mine_return = 1;

	while(c != 0)
	{
		goal.target_pose.pose.position.x = x3[q]* 0.3048;
        	goal.target_pose.pose.position.y = y3[q]* 0.3048;
        	goal.target_pose.pose.orientation.z = 0;
        	goal.target_pose.pose.orientation.w = 1.0;
		ac.sendGoal(goal);
		ROS_INFO("Going to waypoint <%d> ( %.2f, %.2f)", waypoint, x[waypoint], y[waypoint]);
		
		while(1)
		{
			ros::spinOnce();
			mine_return = mine();
			
			//check if the waypoint is achieved and keep checking if the mine is found then we go to the next waypoint
			if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && mine_return == 1)
			{
				ROS_INFO("Acheived waypoint <%d> ( %.2f, %.2f)", waypoint, x[waypoint], y[waypoint]);
				q++;
				c--;
				break;
			}
			//mine has been found, here we will change the state to trash and return however after the integration of arms we will set the state to armed
			else if(mine_return == 0)
			{
				q++;
				c--;
				//set the state to armed
				state = 1;
				return;
			}
		}
	}
	
	ROS_INFO("No mine found\n");
	return;
}

//This function checks the the state of the machine and calls a function that executes the process the robot is supposed to do.
void check_machine_status()
{
//set up three diffrent cases and call the functions in a switch statement 

	switch(state)
	{		
		//HUNT
		case 0:
			//call function which: goes to waypoints to grab known mines
			hunt_state();
			break;
		//ARMED
		case 1:
			//call to grab the mine from servo
			armed_state();
			break;
		//TRASH
		case 2:
			//call the trash function
			trash_state();
			break;
		//SEARCH
		case 4:
			//this function will find it's way to an unknown mine 1
			unknown_mine_1();
			break;
		case 5:
			//this function will take unknown mine to the trash site 
			unknwown_trash1();
			break;
		case 6:
			//this function will find it's way to an unknown mine 2
			unknown_mine_2();
			break;
		case 7: 
			//this function will take unknown mine to the trash site 
			unknwown_trash2();
			break;
		case 8:
			//this function will find it's way to an unknown mine 3
			unknown_mine_3();
			break;
	}
	return;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "challenge_state_machines");
	MoveBaseClient ac("move_base", true);

	ros::NodeHandle n;
	ros::Subscriber lidar1_sub;
	ros::Subscriber lidar2_sub;

	//liadar subsciber setup
	lidar1_sub = n.subscribe("lidar1", 20, lidar1Callback);
	lidar2_sub = n.subscribe("lidar2", 20, lidar2Callback);

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
	waypoint = 0;

	while(1)
	{
		check_machine_status();
		/*if(trash_hit == 6)
		{
		break;
		} */
	}

	ros::spin();
	return 0;
}

