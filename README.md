Robo Project Two

In this project I used what I learned about how to control a robot with six degrees of freedom to perform pick and place actions using inverse kinematics. I had the pleasure of using Gazebo in order to simulate the manipulation of a 6 DOF KR210 robot. The task was for the robot to oick up a cyclinder from a shelf and place it in a bin near to the robot. By manipulating some of the provided code, the position of the cylinders on the shelf could either be randomly chosen or not. For this exercise they position of the cylinders was randomly chosen. During the completion of this project I had the pleasure of deep diving into forward and inverse kinematics in order to calculate motor angles, joint angles, and speeds while performing the simulated tasks. 

 ![alt tag](https://d17h27t6h515a5.cloudfront.net/topher/2017/July/5975d719_fk/fk.png)
 





Rubric Component

    Kinematics
    
    ![alt tag](https://d17h27t6h515a5.cloudfront.net/topher/2017/July/5975d719_fk/fk.png)
    
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
    
    Actual electronic project implementation
    
    IK_server.py with properly commented python code for calculating inverse kinematics based on previous kinematic analysis
    
    80% pick and place cycles with screenshots
    
    Tracking the planned trajectory
    
    Successfully completing pick and place
    


![alt tag](https://github.com/BabaOlude/Robo-Project-Two/blob/master/misc_images/Pick%20and%20Place%201.png)

![alt tag](https://github.com/BabaOlude/Robo-Project-Two/blob/master/misc_images/Pick%20and%20Place%201.png)

![alt tag](https://github.com/BabaOlude/Robo-Project-Two/blob/master/misc_images/Pick%20and%20Place%201.png)

![alt tag](https://github.com/BabaOlude/Robo-Project-Two/blob/master/misc_images/Pick%20and%20Place%201.png)

Code

    Code Explanation
    
Results Discussion

    What Worked

    What Didn't Work

    Future Improvements
