Driver Description 
==================

USB Protocol
------------

A repository describing the communication protocol was found here: https://github.com/ccourson/LewanSoul-xArm
From there, we explain here the way to communicate with the robot.

.. raw:: html

    <table>
    <thead><tr><th></th><th>id</th><th>header</th><th>length</th><th>command</th><th>parameters</th></tr></thead>
    <tbody>
    <tr><td><b>Bytes</b></td><td align="middle">1</td><td align="middle">2</td><td align="middle">1</td><td align="middle">1</td><td align="middle">0 or more</td></tr>
    <tr><td><b>Comments</b></td><td>Any number.</td><td>Always 0x5555.</td><td>Here to end.</td><td>See <i>commands</i>.</td><td>See <i>commands</i>.</td></tr>
    </tbody>
    </table>


Commands are essentially request packets embedded into USB HID reports. Requests and Reqponses are described in the following syntax:

* Each field is seperated by a space.
* Each field is described by the type in parentheses.
* Curly braces denote that their content may be repeated more than once.


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
    
    **These commands will affect all attached servos. Use these commands with only one servi attached.**
    BusServoInfoWrite    27  (byte)id (ushort)pos_min (ushort)pos_max (ushort)volt_min
                             (ushort)volt_max (ushort)temp_max (byte)led_status
                             (byte)led_warning
    BusServoInfoRead     28  -none-; (byte)id (ushort)pos_min (ushort)pos_max (ushort)volt_min
                             (ushort)volt_max (ushort)temp_max (byte)led_status
                             (byte)led_warning (byte)dev_offset (ushort)pos (byte)temp
                             (ushort)volt

Communication with the control board:
https://github.com/ccourson/LewanSoul-xArm

Driver python file modified from:
From https://gist.github.com/maximecb/7fd42439e8a28b9a74a4f7db68281071
Author: Maxime Chevalier-Boisvert

Hardware Interface
==================

URDFs
=====
https://grabcad.com/library/lewansoul-6dof-robot-arm-1

RVIZ-Moveit!
============




