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


Hardware Interface
==================

URDFs
=====
https://grabcad.com/library/lewansoul-6dof-robot-arm-1

RVIZ-Moveit!
============




