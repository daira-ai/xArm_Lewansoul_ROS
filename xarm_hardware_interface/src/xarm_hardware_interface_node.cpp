#include <xarm_hardware_interface/xarm_hardware_interface.h>
#include <ros/callback_queue.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "xarm_hardware_interface");
    ros::CallbackQueue ros_queue;

    ros::NodeHandle nh;
    nh.setCallbackQueue(&ros_queue);
    xarm_hardware_interface::xarmHardwareInterface rhi(nh);
    usleep(5*1000000);
    

    ros::MultiThreadedSpinner spinner(0);
    spinner.spin(&ros_queue);
    return 0;
}