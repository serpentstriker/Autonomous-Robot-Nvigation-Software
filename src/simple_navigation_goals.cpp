#include <ros/ros.h>
#include <iostream>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <math.h>



typedef struct Quaternion {
    double w, x, y, z;
} Quaternion;

Quaternion ToQuaternion(double yaw, double pitch, double roll, Quaternion q) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}


// Creates a convenience typedef for a SimpleActionClient that allows us to communciate actions that align to the MoveBaseAction action interface
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
        MoveBaseClient;

int main(int argc, char** argv){

    double PI = 3.142;

    ros::init(argc, argv, "simple_navigation_goals");

	ros::NodeHandle nv;
	ros::Publisher vel_pub = nv.advertise <geometry_msgs::Twist> ("/cmd_vel", 100);
	geometry_msgs::Twist vel_msg;

    //tell the action client that we want to spin a thread by default
    /* 
    An action client used to communicate with action called "move_base" that adheres to the MoveBaseAction Interface.
    Tells the action client to start a thread to call ros::spin() so that ROS callbacks will be passed as "true" as    the second arguement of the MoveBaseClient constructor.	
    */
    MoveBaseClient ac("move_base", true);

    /*wait for the action server to come up
    Begin processing the goals when the action server reports that it has come up
    */

    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    // Creating a goal to send to move_base using the move_base_msgs::MoveBaseGoal message type
    move_base_msgs::MoveBaseGoal goal;

    //waypoint 1

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 11 * 0.3042;
    goal.target_pose.pose.position.y = 11 * 0.3042;
    goal.target_pose.pose.orientation.z = 1;
    goal.target_pose.pose.orientation.w = 1.0;
    ROS_INFO("Sending goal 1");
    
    // Call to ac.SendGoal - pushes the goal out over the wire to the move_base node for processing
    ac.sendGoal(goal);
    ac.waitForResult();

    goal.target_pose.pose.position.x = 3 * 0.3042;
    goal.target_pose.pose.position.y = 11 * 0.3042;
    goal.target_pose.pose.orientation.z = 1;
    goal.target_pose.pose.orientation.w = 1.0;
    ROS_INFO("Sending angle");
    
	Quaternion q;
	q = ToQuaternion(3.142, 0, 0, q);

	goal.target_pose.pose.orientation.w = q.w;
     goal.target_pose.pose.orientation.x = q.x;
     goal.target_pose.pose.orientation.y = q.y;
     goal.target_pose.pose.orientation.z = q.z;
     
     ROS_INFO("Sending goal");
     ac.sendGoal(goal);

     ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("rotated");
    else
        ROS_INFO("The robot failed to move forward 1.4 meters for some reason");



ROS_INFO("Sending goal");
     ac.sendGoal(goal);

     ac.waitForResult();
	ROS_INFO("Odom initiated to backup");
		ROS_INFO("Go staraight");
			//now we will back up a little bit to not clog the mine in the robot
			vel_msg.linear.x = 0.1;
			vel_pub.publish(vel_msg);
			//half send delay should be okay
			ros::Duration(6).sleep();
			//now lets set the velocity back to 0
			vel_msg.linear.x = 0;
			vel_pub.publish(vel_msg);
			ROS_INFO("Odom terminated");

			ROS_INFO("turn");
			//now we will back up a little bit to not clog the mine in the robot
			vel_msg.angular.z = -1;
			vel_pub.publish(vel_msg);
			//half send delay should be okay
			ros::Duration(1.6).sleep();
			//now lets set the velocity back to 0
			vel_msg.angular.z = 0;
			vel_pub.publish(vel_msg);
			ROS_INFO("Odom terminated");
			
			ROS_INFO("Go staraight");
			//now we will back up a little bit to not clog the mine in the robot
			vel_msg.linear.x = 0.1;
			vel_pub.publish(vel_msg);
			//half send delay should be okay
			ros::Duration(6).sleep();
			//now lets set the velocity back to 0
			vel_msg.linear.x = 0;
			vel_pub.publish(vel_msg);
			ROS_INFO("Odom terminated");

			ROS_INFO("turn");
			//now we will back up a little bit to not clog the mine in the robot
			vel_msg.angular.z = -1;
			vel_pub.publish(vel_msg);
			//half send delay should be okay
			ros::Duration(1.6).sleep();
			//now lets set the velocity back to 0
			vel_msg.angular.z = 0;
			vel_pub.publish(vel_msg);
			ROS_INFO("Odom terminated");


//waypoint 1
    goal.target_pose.pose.position.x = 13 * 0.3042;
    goal.target_pose.pose.position.y = 5 * 0.3042;
    goal.target_pose.pose.orientation.z = 0;
    goal.target_pose.pose.orientation.w = 1.0;
    ROS_INFO("Sending goal 1");
    
    // Call to ac.SendGoal - pushes the goal out over the wire to the move_base node for processing
    ac.sendGoal(goal);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the robot moved 1.4 meters forward - Achieved Goal 11111");
    else
        ROS_INFO("The robot failed to move forward 1.4 meters for some reason");

    return 0;
}
