About
===================



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

#. Install dependencies as mentioned in hidapi repository

   .. prompt:: bash $

      sudo apt-get install libudev-dev libusb-1.0-0-dev libfox-1.6-dev
      sudo apt-get install autotools-dev autoconf automake libtool

#. Run all the commands to build as mentioned in the readme of the repository

   .. prompt:: bash $

      ./bootstrap
      ./configure
      make
      sudo make install 
