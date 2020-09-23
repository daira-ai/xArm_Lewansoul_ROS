Driver Description 
==================

USB Protocol
------------

A repository describing the communication protocol was found here: https://github.com/ccourson/LewanSoul-xArm.

From there, we took the information about the protocol to communicate with the robot.

A packet transmitted to the xArm will have the following format:

.. raw:: html

    <table>
    <thead><tr><th></th><th>id</th><th>header</th><th>length</th><th>command</th><th>parameters</th></tr></thead>
    <tbody>
    <tr><td><b>Bytes</b></td><td align="middle">1</td><td align="middle">2</td><td align="middle">1</td><td align="middle">1</td><td align="middle">0 or more</td></tr>
    <tr><td><b>Comments</b></td><td>Any number.</td><td>Always 0x5555.</td><td>Here to end.</td><td>See <i>commands</i>.</td><td>See <i>commands</i>.</td></tr>
    </tbody>
    </table>


Commands are essentially request packets embedded into USB HID reports. Requests and responses are described in the following syntax:

* Each field is seperated by a space.
* Each field is described by the type in parentheses.
* Curly braces denote that their content may be repeated more than once.

The following are the set of available requests:

::

    ServoMove             3  (byte)count (ushort)time { (byte)id (ushort)position }
    GroupRunRepeat        5  (byte)group[255=all] (byte)times 
    GroupRun              6  (byte)group (ushort)count[0=continuous]
    GroupStop             7  -none-
    GroupErase            8  (byte)group[255=all]
    GroupSpeed           11  (byte)group (ushort)percentage
    xServoOffsetWrite    12  *** not sure
    xServoOffsetRead     13  *** not sure
    xServoOffsetAdjust   14  *** not sure
    GetBatteryVoltage    15  -none-; (ushort)millivolts
    ServoOff             20  (byte)count { (byte)id }
    ServoPositionRead    21  (byte)count { (byte)id }; (byte)count { (byte)id (ushort)position }
    ServoPositionWrite   22  (byte)count { (byte)id (ushort)position }
    ServoOffsetRead      23  (byte)count { (byte)id }; (byte)count { (byte)id (sbyte)offset }
    ServoOffsetWrite     24  (byte)id (sbyte)offset
    BusServoMoroCtrl     26  (byte)id (byte)??? (ushort)speed
    
Taking this information, we have created files to control the robot in python and c++. We have subsequently used the c++ implementation to create the following robot interfaces but we have left the python one in case it is helpful for someone.

Python driver
*************
We found the following `implementation in python for this robot <https://gist.github.com/maximecb/7fd42439e8a28b9a74a4f7db68281071>`_ (Author: Maxime Chevalier-Boisvert). We modified it and you can find it `here <https://github.com/diestra-ai/xArm_Lewansoul_ROS/blob/melodic-devel/xarm_hardware_interface/scripts/controller.py>`_. 

To run it, turn the robot on and run the following:
   
.. prompt:: bash $

  sudo -s  
  python xarm_hardware_interface/scripts/controller.py 
  
There are some commands that will be executed as examples of how to control the robot. You should see the robot doing the following movements:
* TODO: do a video

C++ driver
**********
Starting from the python implementation, we have created the c++ version of it that you can find `here <https://github.com/diestra-ai/xArm_Lewansoul_ROS/blob/melodic-devel/xarm_hardware_interface/src/xarm.cpp>`_

Since there is not information about the joint limits in the robot manual, we have manually found out the values in units and calculated their equivalent in radians by using the Lewansoul mobile app.

+-------+---------------------------+
| Joint |           Units           |
|       +------+-----+-------+------+
|       | Min  | Max | -pi/2 | pi/2 |
+-------+------+-----+-------+------+
|     2 |   50 | 995 |   200 |  980 |
+-------+------+-----+-------+------+
|     3 |  100 | 950 |   140 |  880 |
+-------+------+-----+-------+------+
|     4 |   50 | 950 |   130 |  870 |
+-------+------+-----+-------+------+
|     5 |  135 | 950 |   140 |  880 |
+-------+------+-----+-------+------+
|     6 |   50 | 900 |    90 |  845 |
+-------+------+-----+-------+------+

+-------+----------------------------+
| Joint |           Radians          |
|       +--------------+-------------+
|       | Min          | Max         |
+-------+--------------+-------------+
|     2 |  -2.17494876 |  1.63121157 |
+-------+--------------+-------------+
|     3 | -1.740612146 |  1.86797401 |
+-------+--------------+-------------+
|     4 | -1.910427965 | 1.910427965 |
+-------+--------------+-------------+
|     5 | -1.592023304 |  1.86797401 |
+-------+--------------+-------------+
|     6 | -1.737238322 |  1.79965407 |
+-------+--------------+-------------+


Hardware Interface
==================
We used `this following tutorial <https://www.slaterobotics.com/blog/5abd8a1ed4442a651de5cb5b/how-to-implement-ros_control-on-a-custom-robot>`_ to create the ros_control interfaces for this robot. You can find our implementation `here <https://github.com/diestra-ai/xArm_Lewansoul_ROS/tree/melodic-devel/xarm_hardware_interface>`_.

URDF
=====

Meshes were taken from  https://grabcad.com/library/lewansoul-6dof-robot-arm-1. Different links were exported as single files using Solidworks. The origin and orientation were changed using Blender according with the axis of rotation of the real robot. Joints were defined in the `URDF file <https://github.com/diestra-ai/xArm_Lewansoul_ROS/blob/melodic-devel/xarm_description/urdf/xarm.urdf>`_. 

.. warning::
   At the moment, there is no information about inertia.  

RVIZ-MoveIt!
============

Running ``xarm.launch`` file will launch MoveIt! and RVIZ integration that will allow you to plan trajectories. In this case the joints are controlled using trayectory controller. 

.. prompt:: bash $

   roslaunch xarm_launch xarm.launch
   
.. figure:: ../img/xarm_RVIZ.png
   :width: 50%
   :align: center
   :alt: xArm in RVIZ Interface

We have integrated xArm with MoveIt! using MoveIt! Setup Assistant. `Here <https://github.com/diestra-ai/xArm_Lewansoul_ROS/tree/f_documentation/xarm_moveit_config>`_. you can find the Moveit! configuration and the srdf file  `here <https://github.com/diestra-ai/xArm_Lewansoul_ROS/blob/f_documentation/xarm_moveit_config/config/xarm.srdf>`_. 

.. Note::
   You can control the robot using only position control using the following launch file: 
   
   .. prompt:: bash $

      roslaunch xarm_hardware_interface xarm_position_controller.launch
   
   








