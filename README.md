Robo Project Two

In this project I used what I learned about how to control a robot with six degrees of freedom to perform pick and place actions using inverse kinematics. I had the pleasure of using Gazebo in order to simulate the manipulation of a 6 DOF KR210 robot. The task was for the robot to oick up a cyclinder from a shelf and place it in a bin near to the robot. By manipulating some of the provided code, the position of the cylinders on the shelf could either be randomly chosen or not. For this exercise they position of the cylinders was randomly chosen. During the completion of this project I had the pleasure of deep diving into forward and inverse kinematics in order to calculate motor angles, joint angles, and speeds while performing the simulated tasks. 

 ![alt tag](https://d17h27t6h515a5.cloudfront.net/topher/2017/July/5975d719_fk/fk.png)

 


    
Robot Figures
 ![alt tag](https://github.com/BabaOlude/Robo-Project-Two/blob/master/misc_images/Pick%20and%20Place%201.png)



 ![alt tag](https://github.com/BabaOlude/Robo-Project-Two/blob/master/misc_images/Pick%20and%20Place%202.png)    
Pen and paper figures

 ![alt tag](https://github.com/BabaOlude/Robo-Project-Two/blob/master/misc_images/Written%20Work%20First%20Page.png)

 ![alt tag](https://github.com/BabaOlude/Robo-Project-Two/blob/master/misc_images/Written%20Work%20Second%20Page.png)
 
 ![alt tag](https://github.com/BabaOlude/Robo-Project-Two/blob/master/misc_images/Written%20Work%20Third%20Page.png)
 
  ![alt tag]( https://github.com/BabaOlude/Robo-Project-Two/blob/master/misc_images/Written%20Work%204.png)

    

    

    
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
    
    
Tracking the planned trajectory
    
Successfully completing pick and place
    
Running IK_Server.py
Please see IK_Server file for code
    
Results Discussion

    What Didn't Work
    - The only thing that didn't work for me had to do with my own learning curve. I couldn't figure out how to make the screem bigger on my VM. My mentor suggested another VM but because of time constraints I haven't been able to get that other VM going which would allow me to view the simulation on a full screen.
    What Worked
    - Everything else
    Future Improvements
    - I really enjoyed the project and I didn’t see anything that I would improve. It was a great project.

