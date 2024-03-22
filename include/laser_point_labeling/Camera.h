#include "laser_point_labeling/headers.h"

class Camera
{
    private:
        ros::NodeHandle nh_cam;
        ros::Subscriber img_sub;

        std::mutex mtx_img;
    public:
        void Subscribe_img();

        void Image_callback(const sensor_msgs::ImageConstPtr& _msg);

    Camera()
    {
    }
    ~Camera()
    {

    }
};

void Camera::Subscribe_img(){
    img_sub = nh_cam.subscribe<sensor_msgs::Image>("/camera/color/image_raw", 10, Image_callback, this);
}

void Camera::Image_callback(const sensor_msgs::ImageConstPtr& _img){
    cv_bridge::CvImagePtr cv_ptr;

    mtx_img.lock();

    try
    {
        cv_ptr = cv_bridge::toCvCopy(_img, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
        return;
    }

    cv::Mat image;
    cv_ptr->image.copyTo(image);

    mtx_img.unlock();
}