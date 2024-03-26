#include "laser_point_labeling/Img_proc.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_point_label");
    
    ros::NodeHandle nh;

    Img_proc img_proc;

    img_proc.subscribe();

    ros::spin();
}