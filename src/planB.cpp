// ROS library
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Int16.h>

#define right 0.7071
#define left -0.7071

int lidardata1, lidardata2;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
        MoveBaseClient;

float x[] = {1.22, 2.44, 2.44, 3.36, 3.36, 2.75, 2.75, 1.53, 1.53, 1.835, 1.835, 2.75, 2.75, 0.61, 0.61, 0.915, 0.915};
float y[] = {0.31, 0.31, 1.53, 1.53, 1.835, 1.835, 3.66, 3.66, 3.36, 3.36, 3.66, 3.66, 2.75, 2.75, 1.53, 1.53, 1.835};
float z[] = {0, left, right, left, left, right, left, right, left, 0, right, right, left, left, right, right, left};

int i = 17;
int j = 0;
double PI = 3.142;

// Creating a goal to send to move_base using the move_base_msgs::MoveBaseGoal message type
move_base_msgs::MoveBaseGoal goal;

MoveBaseClient ac("move_base", true);

void lidar1Callback(const std_msgs::Int16::ConstPtr& msg1)
{
	lidardata1 = msg1->data;
}

void lidar2Callback(const std_msgs::Int16::ConstPtr& msg2)
{
	lidardata2 = msg2->data;
}

void sendgoal()
{	
	goal.target_pose.pose.position.x = x[j];
        goal.target_pose.pose.position.y = y[j];
        //goal.target_pose.pose.orientation.z = z[j];
        goal.target_pose.pose.orientation.w = 1.0;
        ROS_INFO("Sending goal %d", j);
	ac.sendGoal(goal);
}

void trash()
{
   	// changes the wp when mine is found
        goal.target_pose.pose.position.x = 3.9624;
        goal.target_pose.pose.position.y = 0.305;
        //goal.target_pose.pose.orientation.z = z[j];
        goal.target_pose.pose.orientation.w = 1.0;
        ROS_INFO("Sending goal TRASH SITE");
}

int mine()
{
    if((lidardata1<100) && ((lidardata2-lidardata1)>200))
    {
        ROS_INFO("Mine found");
        return 0;
    }
    else
    {
        ROS_INFO("lidardata1: %d lidardata2: %d\n", lidardata1, lidardata2);
        return 1;
    }
}


int test = 1;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planB");
    ros::NodeHandle n;
    ros::Subscriber lidar1_sub;
    ros::Subscriber lidar2_sub;

    lidar1_sub = n.subscribe("lidar1", 1, lidar1Callback);
    lidar2_sub = n.subscribe("lidar2", 1, lidar2Callback);

    int mine_return;

  
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    while(i)
    {
        ros::spinOnce();
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
	sendgoal();

        // Call to ac.SendGoal - pushes the goal out over the wire to the move_base node for processing
        //ac.sendGoal(goal);

        while(1)
	{
            ros::spinOnce();
	    mine_return = mine();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && mine_return == 1)
	    {
                ROS_INFO("Acheived waypoint %d  %.2f %.2f ", j, x[j], y[j]);
		i--;
		j++;
                break;
            }
            else if((mine_return == 0))
	    {
		ROS_INFO("Mine Found at %.2f %.2f\n", (x[j] -0.305), (y[j] -0.305));
		i--;
		j++;
		//sendgoal();
		trash();
		ac.sendGoal(goal);
		while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED){}
		i--;
		j++;
		break;
	    }
	    else
	    {
	    	ROS_INFO("On the way to the goal\n");
            }		
        }
    }

    ROS_INFO("Final Position %d  %.2f %.2f", j, x[j], y[j]);

    ros::spin();
    return 0;
}
