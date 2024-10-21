# Restaurant_Robot_Application
Package contains algorithm which manages the process of cafe robot to take orders from the kitchen and delivers to the respective tables and returns to home position.

**PROCESS FLOW CHART:**

![Process_Flow_Chart](https://github.com/user-attachments/assets/85e6bc38-22bc-4deb-a11a-fcbf7ab66f30)


**INSTALLATION**

**Clone the repository**

    *cd catkin_ws/src
    *git clone https://github.com/santhosh2323/Restaurant_Robot_Application.git
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

                                                                      **********************
