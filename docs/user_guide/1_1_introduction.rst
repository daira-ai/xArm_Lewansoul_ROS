Install
===================

System requirements
--------------------

* Ubuntu 18.04
* ROS Melodic

Install package dependencies
--------------------

.. prompt:: bash $
     
   sudo apt-get install libhidapi-hidraw0 libhidapi-libusb0
   pip install --user hid
   sudo apt-get install python-dev libusb-1.0-0-dev libudev-dev
   sudo pip install --upgrade setuptools
   sudo pip install hidapi

Install HIDAPI
--------------------

#. Clone hidapi from  https://github.com/libusb/hidapi

#. Install dependencies as mentioned in hidapi repository:

   .. prompt:: bash $

      sudo apt-get install libudev-dev libusb-1.0-0-dev libfox-1.6-dev
      sudo apt-get install autotools-dev autoconf automake libtool

#. Run all the commands to build as mentioned in the readme of the repository:

   .. prompt:: bash $

      ./bootstrap
      ./configure
      make
      sudo make install 
      
Build xArm_Lewansoul_ROS
--------------------

#. Clone this repository

   .. prompt:: bash $
   
      git clone https://github.com/diestra-ai/xArm_Lewansoul_ROS.git
      
#. Build

   .. prompt:: bash $
   
      catkin_make
      

Run 
====

#. Connect the robot to any USB port of your computer 

#. Turn the robot on.
   You should see the pink and blue lights on the control card. 
   
#. To control the robot with code you sould be in sudo mode
   
   .. prompt:: bash $
   
      sudo -s

#. Run the following script to check the computer is  able to detect and control the robot.
   You should be in the folder of xArm_Lewansoul_ROS
   
   .. prompt:: bash $
   
      python xarm_hardware_interface/scripts/controller.py 
      
   * If the robot is correctly detected you should see the following output:
   
     .. figure:: ../img/python_output.png
        :width: 100%
        :align: left
   
   
     and robot should move to a vertical position and then open and close the gripper followed by rotating its second and third joints.
    
   * If the robot is not detected the following output would appear:
   
     .. figure:: ../img/python_output_error.png
        :width: 100%
        :align: left
    
     in this case check that the robot is on and that you are running from sudo mode, or try another USB port or cable. Also maybe restart the system.  
      
     With this python file you can use different functions to control the robot. 
   
#. Control the robot using ROS
 
   To have the robot running in ROS, launch the following
        
   .. prompt:: bash $
    
      roslaunch xarm_launch xarm.launch
     
   
       
       
    
 

   

   

      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
