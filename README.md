# Restaurant_Robot_Application
Package contains algorithm which manages the process of cafe robot to take orders from the kitchen and delivers to the respective tables and returns to home position.

**PROCESS FLOW CHART**

![process_flow_chart](https://github.com/user-attachments/assets/f901d985-4de8-4bf7-9cef-b3f005d16874)

**SIMULATION ENVIRONMENT**

 In this part turtlebot3_world simulation is utilized and locations(home,kitchen,tables) have been pre-defined as per the below picture.


![Simualtion_Environment](https://github.com/user-attachments/assets/0428b632-5338-48a3-a1f2-1a6f9608cafa)


**PREREQUISITES**

ROS-Noetic

Turtlebot3

Python3

**INSTALLATION**

**Clone the repository**

      cd catkin_ws/src
      git clone https://github.com/santhosh2323/Restaurant_Robot_Application.git
      
Cut the resro_pkg inside the Restaurant_Robot_Application folder and paste inside the src folder:

      cd ..
      catkin_make

**RUNNING**

     roslaunch turtlebot3_gazebo turtlebot3_world.launch

     roslaunch turtlebot3_navigation turtlebot3_navigation.launch

     rosrun resro_pkg resro_manager.py

**Giving Orders**

     rostopic pub /order_list resro_pkg/StringList "data: ['table_1', 'table_2', 'table_3']"

**Kitchen confirmation**

     rostopic pub /kitchen_confirmation std_msgs/String "data: ''"

**Table confirmation**

     rostopic pub /table_confirmation std_msgs/String "data: ''"

**Cancelling Orders**

     rosservice call /cancel/order "msg: 'table_2'" 


*** END ***
