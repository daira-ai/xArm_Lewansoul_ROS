#include "ros/ros.h"
#include <stdexcept>
#include <xarm_hardware_interface/xarm.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#define MAX_STR 255
#define PI 3.14159265359

using std::string;

namespace xarm
{
	xarm::xarm()
	{		
		// Initialize the hidapi library
		if (hid_init())
			return;

		int found=0;
		printDeviceInformation();
		devs = hid_enumerate(0x0, 0x0);
		cur_dev = devs;

		while (cur_dev) {
					
			std::wstring ws(cur_dev->product_string);
			string product(ws.begin(), ws.end());
		
			if (product=="LOBOT")
			{
				ROS_INFO("LOBOT found \n");
				found=1;
				break;
			}
			cur_dev = cur_dev->next;
		}
		if (found==0)
		{
			ROS_ERROR("LOBOT not found, make sure it is power on \n");
			throw std::exception();
		}

		handle = hid_open_path(cur_dev->path);
		
		if (!handle) {
			ROS_ERROR("unable to open device\n");
 			throw std::exception();
		}
		ROS_INFO("Device opened \n");
		hid_free_enumeration(devs);

		//Dictionary of joint_names to joint_id
		joint_name_map.insert(std::make_pair("xarm_1_joint" , 1));
		joint_name_map.insert(std::make_pair("xarm_2_joint" , 2));
		joint_name_map.insert(std::make_pair("xarm_3_joint" , 3));
  		joint_name_map.insert(std::make_pair("xarm_4_joint" , 4));
		joint_name_map.insert(std::make_pair("xarm_5_joint" , 5));
		joint_name_map.insert(std::make_pair("xarm_6_joint" , 6));
		
		// Gripper
		matrix_unit_transform["xarm_1_joint"][0][0]=-360;
		matrix_unit_transform["xarm_1_joint"][0][1]=700;

		// Arm joints
		matrix_unit_transform["xarm_2_joint"][0][0]=200;
		matrix_unit_transform["xarm_2_joint"][0][1]=980;
		matrix_unit_transform["xarm_3_joint"][0][0]=140;
		matrix_unit_transform["xarm_3_joint"][0][1]=880;
		matrix_unit_transform["xarm_4_joint"][0][0]=870;
		matrix_unit_transform["xarm_4_joint"][0][1]=130;
		matrix_unit_transform["xarm_5_joint"][0][0]=140;
		matrix_unit_transform["xarm_5_joint"][0][1]=880;
		matrix_unit_transform["xarm_6_joint"][0][0]=90;
		matrix_unit_transform["xarm_6_joint"][0][1]=845;
	}

	xarm::~xarm()
	{
		hid_close(handle);

		/* Free static HIDAPI objects. */
		hid_exit();
	}

	void xarm::printDeviceInformation()
	{
		devs = hid_enumerate(0x0, 0x0);
		cur_dev = devs;	
		while (cur_dev) {
			printf("Device Found\n  type: %04hx %04hx\n  path: %s\n  serial_number: %ls", cur_dev->vendor_id, cur_dev->product_id, cur_dev->path, cur_dev->serial_number);
			printf("\n");
			printf("  Manufacturer: %ls\n", cur_dev->manufacturer_string);
			printf("  Product:      %ls\n", cur_dev->product_string);
			printf("  Release:      %hx\n", cur_dev->release_number);
			printf("  Interface:    %d\n",  cur_dev->interface_number);
			printf("\n");
		
			cur_dev = cur_dev->next;
		}
	}

	int xarm::convertRadToUnit(std::string joint_name, double rad)
	{
		int unit;
		double m= (matrix_unit_transform[joint_name][0][1]-matrix_unit_transform[joint_name][0][0])/(PI);
		double b = matrix_unit_transform[joint_name][0][1] - (m*PI/2);
		unit = (m*rad) + b;
		return unit;
	}

	double xarm::convertUnitToRad(std::string joint_name, int unit)
	{
		double rad;
		double m= (PI)/(matrix_unit_transform[joint_name][0][1]-matrix_unit_transform[joint_name][0][0]);
		double b = (PI/2) - (m*matrix_unit_transform[joint_name][0][1]);
		rad = (m*unit) + b;
		return rad;
	}

	double xarm::convertUnitToGripperRatio(int unit)
	{
		double ratio;
		//ratio= 1.0 - ((float(unit) - matrix_unit_transform["xarm_1_joint"][0][0])/(matrix_unit_transform["xarm_1_joint"][0][1]-matrix_unit_transform["xarm_1_joint"][0][0]));
		ratio = cos(ConvertUnitToRad(unit))
		printf("Units: %d and Gripper ratio: %f \n", unit, ratio);
		return ratio;
	}

	int xarm::convertGripperRatioToUnit(double ratio)
	{
		int unit;
		// unit = int(((1.0 - ratio) * (matrix_unit_transform["xarm_1_joint"][0][1]-matrix_unit_transform["xarm_1_joint"][0][0])) + matrix_unit_transform["xarm_1_joint"][0][0]);
		unit = convertRadToUnit(acos(ratio))
		printf("Gripper ratio: %f  and units: %d \n", ratio, unit);
		return unit;
	}
	
	std::vector<double> xarm::readJointsPosition(std::vector<std::string> joint_names)
	{
		int res;
		std::vector<double> joint_positions;
		unsigned char buf[65];

		joint_positions.resize(joint_names.size());
		buf[0] = 0x55;
		buf[1] = 0x55;
		buf[2] = 9;
		buf[3] = 21;
		buf[4] = 6;
		buf[5] = 1;
		buf[6] = 2;
		buf[7] = 3;
		buf[8] = 4;
		buf[9] = 5;
		buf[10] = 6;
		res = hid_write(handle, buf, 17);
		
		if (res < 0) {
			printf("Unable to write()\n");
			printf("Error: %ls\n", hid_error(handle));
		}
		
		res = 0;
		while (res == 0) {
			res = hid_read(handle, buf, sizeof(buf));
			if (res == 0)
				printf("waiting...\n");
			if (res < 0)
				printf("Unable to read()\n");
			
			usleep(500*1000);
			
		}

		int id, p_lsb, p_msb, pos, unit, joint_id;
		for (int i=0; i<joint_names.size(); i++){
			joint_id = joint_name_map[joint_names[i]];
			id = buf[2+3*joint_id];
			p_lsb= buf[2+3*joint_id+1];
			p_msb= buf[2+3*joint_id+2];
			unit= (p_msb << 8) + p_lsb;
			if (joint_id == 1){
				joint_positions[i] = convertUnitToGripperRatio(unit);
			}
			else {
				joint_positions[i] = convertUnitToRad(joint_names[i], unit);
			}
			// printf("servo %d in joint_position %f \n", id, joint_positions[i]);
		}

		return joint_positions;
	}
	
	void  xarm::setJointPosition(std::string joint_name, double position_rad, int time=1000)
	{	    
		unsigned char buf[65];
		unsigned char t_lsb,t_msb, p_lsb, p_msb;
		int res;
		int position_unit; 
		
		if (joint_name_map[joint_name] == 1){
			position_unit = int(convertGripperRatioToUnit(position_rad));
		}
		else {
			position_unit = int(convertRadToUnit(joint_name, position_rad));
		}
        t_lsb= time & 0xFF;
		t_msb = time >> 8;
		p_lsb = position_unit & 0xFF;
		p_msb = position_unit >> 8;

		buf[0] = 0x55;
		buf[1] = 0x55;
		buf[2] = 8;
		buf[3] = 0x03;
		buf[4] = 1;
		buf[5] = t_lsb;
		buf[6] = t_msb;
		buf[7] = joint_name_map[joint_name];
		buf[8] = p_lsb;
		buf[9] = p_msb;
		
		res = hid_write(handle, buf, 17);
		
		if (res < 0) {
			printf("Unable to write()\n");
			printf("Error: %ls\n", hid_error(handle));
		}

	} 
	
}
