About
===================

Instalation instructiois

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

Install package dependencies
--------------------

Now we are using hidapi from  https://github.com/libusb/hidapi; it is the same people but seems more updated. 
Steps  to compile hidapi:
Clone hidapi repo from libusb 
Install dependencies as mentioned in the readme of the repository.
Run all the commands to build as mentioned in the readme of the repository.
./bootstrap
./configure
make
Sudo make install 
Include in your cpp or h: #include <hidapi/hidapi.h>
Configure your cMakeList: 
target_link_libraries(xarm_hardware_interface
xarm
hidapi-hidraw
${catkin_LIBRARIES})

