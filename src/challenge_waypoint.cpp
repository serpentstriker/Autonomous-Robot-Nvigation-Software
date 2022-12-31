/*
 Created by 19023 on 2022-06-16.
 Code obtained from the Lab6 document provided by Prof. Vincent Sieben.
 A simple C++ program that emulates the automation method of sending navigation goals to the robot
*/

// ROS library
#include <ros/ros.h>

//Including the ROS action, move_base that accepts goals from clients and attempts to move the robot to the specified position or orientation in the world 
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Creates a convenience typedef for a SimpleActionClient that allows us to communciate actions that align to the MoveBaseAction action interface
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
        MoveBaseClient;

int main(int argc, char** argv){

    double PI = 3.142;

    ros::init(argc, argv, "simple_navigation_goals");

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

    goal.target_pose.pose.position.x = 2.43;
    goal.target_pose.pose.position.y = 0.3;
    goal.target_pose.pose.orientation.z = PI/2;
    goal.target_pose.pose.orientation.w = 1.0;
    ROS_INFO("Sending goal 1");
    
    // Call to ac.SendGoal - pushes the goal out over the wire to the move_base node for processing
    ac.sendGoal(goal);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the robot moved 1.4 meters forward - Achieved Goal 1");
    else
        ROS_INFO("The robot failed to move forward 1.4 meters for some reason");


    //waypoint 2
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 2.43;
    goal.target_pose.pose.position.y = 1.51;
    goal.target_pose.pose.orientation.z = PI/2;
    goal.target_pose.pose.orientation.w = 1.0;
    ROS_INFO("Sending goal 2");

    // Call to ac.SendGoal - pushes the goal out over the wire to the move_base node for processing
    ac.sendGoal(goal);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the robot moved another 1.4 meters forward - Achieved Goal 2");
    else
        ROS_INFO("The robot failed to move forward 1.4 meters for some reason");

    //waypoint 3 -- Infront of the KNOWN MINE 1
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = 3.03;
    goal.target_pose.pose.position.y = 1.51;
    goal.target_pose.pose.orientation.z = PI/2;
    goal.target_pose.pose.orientation.w = 1.0;
    ROS_INFO("Sending goal 3");
    
    // Call to ac.SendGoal - pushes the goal out over the wire to the move_base node for processing
    ac.sendGoal(goal);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the robot moved another 1.4 meter forward - Achieved Goal 3");
    else
        ROS_INFO("The robot failed to move forward 1 meter for some reason");
/*
    //waypoint 4
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = 3.86;
    goal.target_pose.pose.position.y = 1.51;
    goal.target_pose.pose.orientation.z = PI/2;
    goal.target_pose.pose.orientation.w = 1.0;
    ROS_INFO("Sending goal 4");

    // Call to ac.SendGoal - pushes the goal out over the wire to the move_base node for processing
    ac.sendGoal(goal);
    ac.waitForResult();

 //waypoint 5
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = 3.86;
    goal.target_pose.pose.position.y = 2.51;
    goal.target_pose.pose.orientation.z = PI/2;
    goal.target_pose.pose.orientation.w = 1.0;
    ROS_INFO("Sending goal 5");

    // Call to ac.SendGoal - pushes the goal out over the wire to the move_base node for processing
    ac.sendGoal(goal);
    ac.waitForResult();

 //waypoint 6 -- Infront of KNOWN MINE 2
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = 0.85;
    goal.target_pose.pose.position.y = 2.51;
    goal.target_pose.pose.orientation.z = -PI/2;
    goal.target_pose.pose.orientation.w = 1.0;
    ROS_INFO("Sending goal 6");

    // Call to ac.SendGoal - pushes the goal out over the wire to the move_base node for processing
    ac.sendGoal(goal);
    ac.waitForResult();

 //waypoint 7
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = 2.46;
    goal.target_pose.pose.position.y = 2.51;
    goal.target_pose.pose.orientation.z = PI/2;
    goal.target_pose.pose.orientation.w = 1.0;
    ROS_INFO("Sending goal 7");

    // Call to ac.SendGoal - pushes the goal out over the wire to the move_base node for processing
    ac.sendGoal(goal);
    ac.waitForResult();
/*
//waypoint 8 -- Infront of KNOWN MINE 3
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = 2.46;
    goal.target_pose.pose.position.y = 2.81;
    goal.target_pose.pose.orientation.z = PI/2;
    goal.target_pose.pose.orientation.w = 1.0;
    ROS_INFO("Sending goal 8");

    // Call to ac.SendGoal - pushes the goal out over the wire to the move_base node for processing
    ac.sendGoal(goal);
    ac.waitForResult();

 */
   return 0;
}
