#include "laser_point_labeling/Camera.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_point_labeling");

    Camera camera;
    camera.Subscribe_img();
    ros::spin();

    return 0;
}
