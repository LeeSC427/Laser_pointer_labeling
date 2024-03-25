#include "laser_point_labeling/headers.h"
#include "laser_point_labeling/Process.h"

class Camera
{
    private:
        ros::NodeHandle nh_cam;
        ros::Subscriber img_sub;

        std::mutex mtx_img;

        Process cam_process;
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
    img_sub = nh_cam.subscribe<sensor_msgs::Image>("/camera/color/image_raw", 10, &Camera::Image_callback, this);
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

    std::vector<Corner> cam_corner_vec;
    cv::Mat preprocessed_image;
    std::vector<cv::Point2f> contours;
    
    preprocessed_image = cam_process.Preprocessing(image);
    contours = cam_process.find_contour(image, preprocessed_image);
    cam_corner_vec = cam_process.find_corners(image, preprocessed_image, contours);

    cam_process.show_videos(image, preprocessed_image);
}