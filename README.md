Robo Project Two

In this project I used what I learned about how to control a robot with six degrees of freedom to perform pick and place actions using inverse kinematics. I had the pleasure of using Gazebo in order to simulate the manipulation of a 6 DOF KR210 robot. The task was for the robot to oick up a cyclinder from a shelf and place it in a bin near to the robot. By manipulating some of the provided code, the position of the cylinders on the shelf could either be randomly chosen or not. For this exercise they position of the cylinders was randomly chosen. During the completion of this project I had the pleasure of deep diving into forward and inverse kinematics in order to calculate motor angles, joint angles, and speeds while performing the simulated tasks. 

 ![alt tag](https://d17h27t6h515a5.cloudfront.net/topher/2017/July/5975d719_fk/fk.png)
 

 ![alt tag](https://github.com/BabaOlude/Robo-Project-Two/blob/master/misc_images/Pick%20and%20Place%201.png)



 ![alt tag](https://github.com/BabaOlude/Robo-Project-Two/blob/master/misc_images/Pick%20and%20Place%202.png)


 ![alt tag](https://github.com/BabaOlude/Robo-Project-Two/blob/master/misc_images/Written%20Work%20First%20Page.png)
 
 ![alt tag](https://github.com/BabaOlude/Robo-Project-Two/blob/master/misc_images/Written%20Work%20Second%20Page.png)
 
 ![alt tag](https://github.com/BabaOlude/Robo-Project-Two/blob/master/misc_images/Written%20Work%20Third%20Page.png)
 
  ![alt tag]( https://github.com/BabaOlude/Robo-Project-Two/blob/master/misc_images/Written%20Work%204.png)
 


Rubric Component

Kinematics
    
    
    
DH Parameter Table
    
Annotated figure of the robot
    
Pen and paper figure
    
Using the DH paramter to create individual transform matrices about each joint
    
Generalized homogeneous transform between base_link and gripper_link using on end effector pose
    
Handwritten matrices
    
Explanation of how the matrices were created
    
Decoupled inverse kinematics problem into inverse position kinematics and inverse orientation kinematics
    
Derived equations for calculating joint angles
    
Best solution among multiple solutions
    
Project Implementation
-Commands


$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make

$ cd ~/catkin_ws/src
$ git clone https://github.com/udacity/RoboND-Kinematics-Project.git

$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y

$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ sudo chmod u+x target_spawn.py
$ sudo chmod u+x IK_server.py
$ sudo chmod u+x safe_spawner.sh

$ cd ~/catkin_ws
$ catkin_make

$ source ~/catkin_ws/devel/setup.bash

$ echo "export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/models" >> ~/.bashrc

In another terminal


$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ ./safe_spawner.sh

   
IK_server.py with properly commented python code for calculating inverse kinematics based on previous kinematic analysis
    
80% pick and place cycles with screenshots
    
Tracking the planned trajectory
    
Successfully completing pick and place
    




Code
    
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
	#
	#
	# Create Modified DH parameters
	#
	#
	# Define Modified DH Transformation matrix
	#
	#
	# Create individual transformation matrices
	#
	#
	# Extract rotation matrices from the transformation matrices
	#
	#
        ###

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    #
	    #
	    # Calculate joint angles using Geometric IK method
	    #
	    #
            ###

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
    
Results Discussion

    What Didn't Work
    - The only thing that didn't work for me had to do with my own learning curve. I couldn't figure out how to make the screem bigger on my VM. My mentor suggested another VM but because of time constraints I haven't been able to get that other VM going which would allow me to view the simulation on a full screen.
    What Worked
    - Everything else
    Future Improvements
    - I really enjoyed the project and I didn’t see anything that I would improve. It was a great project.

