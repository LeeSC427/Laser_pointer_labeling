#include "laser_point_labeling/ImgProcess.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_point_labeling");
    ros::NodeHandle nh;

    ImgProcess imgproc;

    imgproc.subscribe();

    ros::spin();

    return 0;
}
