#!/usr/bin/env python
# license removed for brevity
import math
import rospy as ros
import sys
import time

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


__author__ = "Gabriel Urbain" 
__copyright__ = "Copyright 2018, IDLab, UGent"

__license__ = "MIT" 
__version__ = "1.0" 
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be" 
__status__ = "Education" 
__date__ = "October 15th, 2018"


class ShapeMove(object):
    """
    This class is an abstract class to control a shape trajectory on the turtleBot.
    It mainly declare and subscribe to ROS topics in an elegant way.
    """

    def __init__(self):

        # Declare ROS subscribers and publishers
        self.node_name = "shape_move"
        self.odom_sub_name = "/odom"
        self.vel_pub_name = "/cmd_vel"
        self.vel_pub = None
        self.odometry_sub = None

        # ROS params
        self.pub_rate = 0.1
        self.queue_size = 2

        # Variables containing the sensor information that can be used in the main program
        self.odom_pose = None

    def start_ros(self):

        # Create a ROS node with a name for our program
        ros.init_node(self.node_name, log_level=ros.INFO)

        # Define a callback to stop the robot when we interrupt the program (CTRL-C)
        ros.on_shutdown(self.stop_robot)

        # Create the Subscribers and Publishers
        self.odometry_sub = ros.Subscriber(self.odom_sub_name, Odometry, callback=self.__odom_ros_sub, queue_size=self.queue_size)
        self.vel_pub = ros.Publisher(self.vel_pub_name, Twist, queue_size=self.queue_size)

    def stop_robot(self):

        # Get the initial time
        self.t_init = time.time()

        # We publish for a second to be sure the robot receive the message
        while time.time() - self.t_init < 1 and not ros.is_shutdown():
            
            self.vel_ros_pub(Twist())
            time.sleep(self.pub_rate)

        sys.exit("The process has been interrupted by the user!")

    def move(self):
        """ To be surcharged in the inheriting class"""

        while not ros.is_shutdown():
            time.sleep(1)

    def __odom_ros_sub(self, msg):

        self.odom_pose = msg.pose.pose

    def vel_ros_pub(self, msg):

        self.vel_pub.publish(msg)



class ShapeMoveOdom(ShapeMove):
    """
    This class implements a semi closed-loop shape trajectory based on relative position control,
    where only odometry is used. HOWTO:
     - Start the sensors on the turtlebot:
            $ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch 
     - Start this node on your computer:
            $ python eced3901_dt1.py odom
    """

    def __init__(self):


        super(ShapeMoveOdom, self).__init__()

        self.pub_rate = 0.1

    #students need to convert from quaternion to RPY: ==============================
    def get_yaw_rotation(self, orientation):

        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        #print roll, pitch, yaw
        return yaw
     
    #students need to create check on angle wrapping: ==============================
    def wrap_angle(self, angle_in):
    
        angle_in = math.fmod(angle_in + math.pi,2*math.pi)
        if (angle_in <= 0):
           angle_in += 2*math.pi           
        return angle_in - math.pi
    
    #students need to limit/adj linear velocity =============================
    def move_of(self, d, speed=0.15):

        x_init = self.odom_pose.position.x
        y_init = self.odom_pose.position.y

        # Set the velocity forward until distance is reached
        while math.sqrt((self.odom_pose.position.x - x_init)**2 + \
             (self.odom_pose.position.y - y_init)**2) < d and not ros.is_shutdown():

            sys.stdout.write("\r [MOVE] The robot has moved of {:.2f}".format(math.sqrt((self.odom_pose.position.x - x_init)**2 + \
            (self.odom_pose.position.y - y_init)**2)) +  "m over " + str(d) + "m")
            sys.stdout.flush()

            msg = Twist()
            msg.linear.x = speed
            msg.angular.z = 0
            self.vel_ros_pub(msg)
            time.sleep(self.pub_rate)

        sys.stdout.write("\n")

    
    #students need to limit/adj angular velocity =============================
    def turn_of(self, a, ang_speed=0.1):

        # Convert the orientation quaternion message to Euler angles
        a_init = self.get_yaw_rotation(self.odom_pose.orientation)
        #print a_init

    #students need to fix the angle handling ========================
    
        # Set the angular velocity forward until angle is reached
        while ( abs(self.wrap_angle(self.get_yaw_rotation(self.odom_pose.orientation) - a_init)) ) < a and not ros.is_shutdown():

            # Check on ranges at wrap conditions
            if ang_speed > 0 and self.wrap_angle(self.get_yaw_rotation(self.odom_pose.orientation)-a_init) < -0.5 :
		break
	    elif ang_speed < 0 and self.wrap_angle(a_init-self.get_yaw_rotation(self.odom_pose.orientation)) < -0.5 :
		break

            # Print status
            sys.stdout.write("\r [TURN] Robot yaw {:.2f}".format(self.get_yaw_rotation(self.odom_pose.orientation)\
                 ) + " from initial {:.2f}".format(a_init) + ", Diff. {:.2f}".format\
                 ((self.wrap_angle(self.get_yaw_rotation(self.odom_pose.orientation)-a_init))) + " over target {:.2f}".format(a) + "rad" )
            sys.stdout.flush()

            msg = Twist()
            msg.angular.z = ang_speed
            msg.linear.x = 0
            self.vel_ros_pub(msg)
            time.sleep(self.pub_rate)

        sys.stdout.write("\n")

    def move(self):

        # Wait that our python program has received its first messages
        while self.odom_pose is None and not ros.is_shutdown():
            time.sleep(0.1)
   

    #students need to fix these angles ===============================

        # Implement main fixed Dead-Reckon coordinate instructions
        
        # Counterclockwise through shape
        self.move_of(1.1)
        self.turn_of(math.pi*1/2, 0.2)
        self.move_of(1.1)
        self.turn_of(math.pi*1/2, 0.2)
        self.move_of(1.1)
        self.turn_of(math.pi*1/2, 0.2)
        self.move_of(1.1)
    
      
        self.stop_robot()




if __name__ == '__main__':

    # Choose the example you need to run in the command line
    if len(sys.argv) > 1:

        if sys.argv[1] == "odom":
            r = ShapeMoveOdom()

        else:
            sys.exit(-1)

    else:
        sys.exit(-1)

    # Listen and Publish to ROS + execute moving instruction
    r.start_ros()
    r.move()
