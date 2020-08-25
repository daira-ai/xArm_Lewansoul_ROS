
#ifndef XARM__H
#define XARM__H
#include <hidapi/hidapi.h>
#include <sstream>
#include <map>

namespace xarm
{
	class xarm
	{
		public:
			xarm();
			~xarm();
			
			std::vector<double> readJointsPosition(std::vector<std::string> joint_names);
			void  setJointPosition(std::string joint_name, double position_rad, int time); 
			double convertUnitToRad(std::string joint_name, int unit);
			int convertRadToUnit(std::string joint_name, double rad);

		private:
			hid_device *handle;
			struct hid_device_info *devs, *cur_dev; 
			void printDeviceInformation();
			int matrix_unit_rad[6][2];
			std::map<std::string, int> joint_name_map;
			std::map<std::string, int[1][2]> matrix_unit_transform;
	};
}

#endif