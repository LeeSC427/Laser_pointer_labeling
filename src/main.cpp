#include "laser_point_labeling/Process.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_point_label");
    ros::NodeHandle nh;
    Process proc;

    proc.subscribe();

    ros::spin();
}