// ROS library
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Int16.h>

#define right 0.7071
#define left -0.7071


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
        MoveBaseClient;

//float x[] = {0.6, 1.2, 1.8, 2.4, 2.4, 2.4, 2.4, 3.0, 3.3, 3.3, 3.6, 3.6, 3.3, 3.3, 3.0, 3.0, 2.4, 1.8, 1.8, 1.2, 1.8, 1.8, 2.4, 3.0, 3.0, 3.0, 2.4, 1.8, 1.2, 0.6, 0.6, 0.6, 0.6, 0.6, 0.9, 0.9, 0.6, 0.6, 0.6, 0.6, 1.2, 1.5, 1.5, 1.5, 1.5, 1.5, 1.2, 0.6, 0.6};
//float y[] = {0.3, 0.3, 0.3, 0.3, 0.6, 1.2, 1.5, 1.5, 1.5, 1.8, 1.8, 2.4, 2.4, 3.0, 3.0, 3.3, 3.3, 3.3, 3.0, 3.0, 3.0, 3.3, 3.3, 3.3, 3.0, 2.7, 2.7, 2.7, 2.7, 2.7, 2.4, 2.1, 1.8, 1.5, 1.5, 1.8, 1.8, 2.1, 2.4, 2.7, 2.7, 2.7, 2.4, 1.8, 1.2, 0.9, 0.9, 0.9, 0.3};

float x[] = {1.22, 2.44, 2.44, 3.36, 3.36, 2.75, 2.75, 3.055, 2.745, 2.745, 0.915, 0.915};
float y[] = {0.31, 0.31, 1.53, 1.53, 1.835, 1.835, 3.36, 3.36, 3.36, 2.75, 2.75, 2.445};
float z[] = {0, left, right, left, left, right, left, left, left, right, left, left};


//int i = 49;
int i = 12;

int j = 0;
double PI = 3.142;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "waypoints");

    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    // Creating a goal to send to move_base using the move_base_msgs::MoveBaseGoal message type
    move_base_msgs::MoveBaseGoal goal;

    while(i)
    {
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = x[j];
        goal.target_pose.pose.position.y = y[j];
        goal.target_pose.pose.orientation.z = z[j];
        goal.target_pose.pose.orientation.w = 1.0;
        ROS_INFO("Sending goal %d", j);

        // Call to ac.SendGoal - pushes the goal out over the wire to the move_base node for processing
        ac.sendGoal(goal);
	ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
           ROS_INFO("Acheived waypoint %d  %.2f %.2f ", j, x[j], y[j]);
	else
	    ROS_INFO("On the way to the goal\n");
            
	i--;
	j++;		
        
    }

    ROS_INFO("Final Position %d  %.2f %.2f", j, x[j-1], y[j-1]);
    return 0;
}
