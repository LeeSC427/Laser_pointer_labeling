#include "laser_point_labeling/Points.h"
#include "laser_point_labeling/Calculate.h"

class Img_Proc
{
    private:
        cv::Scalar low_HSV = cv::Scalar(40,60,50);
        cv::Scalar up_HSV = cv::Scalar(80,255,255);
        std::mutex mtx_img;
        //std::mutex mtx_prev;
    public:
        Calculate calculate;
        ros::NodeHandle nh_img;
        ros::Subscriber img_sub;
        bool initial_corner;
        std::vector<Corner> prev_corners;
        
        cv::Mat Preprocessing(cv::Mat& _img);
        std::vector<cv::Point> find_contour(cv::Mat& _img, cv::Mat& _proc_img);
        std::vector<Corner> find_corners(cv::Mat& _img, cv::Mat& _proc_img, std::vector<cv::Point>& _contours);
        std::vector<Corner> find_initial_corners(std::vector<cv::Point2f>& _corners);
        std::vector<Corner> find_recursive_corners(std::vector<cv::Point2f>& _corners);
        std::vector<Corner> matching_point(std::vector<cv::Point2f>& _corners, std::vector<cv::Point2f>& _moved_corners);

        void draw_corners(cv::Mat& _img,  std::vector<Corner>& _corners);
        void show_videos(cv::Mat& _img, cv::Mat& _proc_img);
        void subscribe();
        void callback(const sensor_msgs::ImageConstPtr& _msg);

    Img_Proc():initial_corner(true){}
    ~Img_Proc(){}
};

cv::Mat Img_Proc::Preprocessing(cv::Mat& _img)
{
    ROS_INFO("PREPROCESSING");
    cv::Mat img_hsv;
    cv::cvtColor(_img, img_hsv, cv::COLOR_BGR2HSV);
    cv::inRange(img_hsv, low_HSV, up_HSV, img_hsv);
    return img_hsv;
}

std::vector<cv::Point> Img_Proc::find_contour(cv::Mat& _img, cv::Mat& _proc_img)
{
    ROS_INFO("FIND CONTOUR");
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<cv::Point> largest_contour;
    cv::findContours(_proc_img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    if(!contours.empty())
    {
        double largest_area = 0.0;
        int largest_contour_index = 0;
        for(int i = 0; i < contours.size(); i++)
        {
            double area = cv::contourArea(contours[i]);
            if(area > largest_area)
            {
                largest_area = area;
                largest_contour_index = 1;
            }
        }
        largest_contour = contours[largest_contour_index];
        cv::drawContours(_img, contours, largest_contour_index, cv::Scalar(0,0,127), 2);
    }
    return largest_contour;
}

std::vector<Corner> Img_Proc::find_corners(cv::Mat& _img, cv::Mat& _proc_img, std::vector<cv::Point>& _contours)
{
    ROS_INFO("FIND CORNERS");
    std::vector<Corner> cur_corners;
    std::vector<cv::Point2f> corners;
    double epsilon = 0.1 * cv::arcLength(_contours, true);
    cv::TermCriteria term_criteria = cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 30, 0.1);
    cv::approxPolyDP(_contours, corners, epsilon, true);
    cv::cornerSubPix(_proc_img, corners, cv::Size(11,11), cv::Size(-1,-1), term_criteria);
    if(corners.size() == 4)
    {
        if(initial_corner)
        {
            cur_corners = find_initial_corners(corners);
        }
        else
        {
            cur_corners = find_recursive_corners(corners);
        }
        if(cur_corners.size() == 4)
        {
            //mtx_prev.lock();
            prev_corners = cur_corners;
            //mtx_prev.unlock();
        }
    }
    else
    {
        //mtx_prev.lock();
        prev_corners.clear();
        //mtx_prev.unlock();
        initial_corner = true;
        ROS_WARN("NUMBER OF CURRENT CORNERS ARE NOT 4");
    }

    return cur_corners;
}

std::vector<Corner> Img_Proc::find_initial_corners(std::vector<cv::Point2f>& _corners)
{
    ROS_INFO("FIND INITIAL CORNERS");
    Corner corner;
    std::vector<Corner> temp_corners;
    std::vector<cv::Point2f> ordered_corners = calculate.ordering(_corners);
    //mtx_prev.lock();
    prev_corners.clear();
    //mtx_prev.unlock();
    for(int i = 0; i < ordered_corners.size(); i++)
    {
        corner.coord = ordered_corners[i];
        corner.label = i + 1;
        temp_corners.push_back(corner);
    }
    initial_corner = false;
    return temp_corners;
}

std::vector<Corner> Img_Proc::find_recursive_corners(std::vector<cv::Point2f>& _corners)
{
    ROS_INFO("FIND RECURSIVE CORNERS");
    cv::Point2f prev_center;
    cv::Point2f cur_center;
    cv::Point2f center_dist;
    std::vector<Corner> temp_corners;
    if(prev_corners.size() == 4)
    {
        std::vector<cv::Point2f> moved_corners;
        prev_center = calculate.find_centroid(prev_corners);
        cur_center = calculate.find_centroid(_corners);
        center_dist = calculate.translation(prev_center, cur_center);
        moved_corners = calculate.move_points(center_dist, _corners);
        temp_corners = matching_point(_corners, moved_corners);
    }
    else
    {
        ROS_WARN("NUMBER OF PREVIOUS CORNERS ARE NOT 4");
        //mtx_prev.lock();
        prev_corners.clear();
        //mtx_prev.unlock();
        initial_corner = true;
    }

    return temp_corners;
}

std::vector<Corner> Img_Proc::matching_point(std::vector<cv::Point2f>& _corners, std::vector<cv::Point2f>& _moved_corners)
{
    ROS_INFO("MATCHING POINT");
    std::vector<Corner> temp_corners;
    Corner corner;
    for(int i = 0; i < _moved_corners.size(); i++)
    {
        double min_dist = std::numeric_limits<double>::infinity();
        int min_index;
        for(int j = 0; j < prev_corners.size(); j++)
        {
            double dist = calculate.get_distance(_moved_corners[i], prev_corners[j].coord);
            if(min_dist > dist)
            {
                min_dist = dist;
                min_index = j;
            }
        }
        ROS_INFO("CORNER %d GOES TO CORNER %d", i, min_index);
        corner.coord = _corners[min_index];
        corner.label = min_index + 1;
        temp_corners.push_back(corner);
    }
    return temp_corners;
}

void Img_Proc::draw_corners(cv::Mat& _img, std::vector<Corner>& _corners)
{
    ROS_INFO("DRAW CORNERS");
    for(int i = 0; i < _corners.size(); i++)
    {
        cv::circle(_img, _corners[i].coord, 3, cv::Scalar(127,0,0), -1);
        cv::putText(_img, std::to_string(_corners[i].label), _corners[i].coord, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,255), 2);
    }
}

void Img_Proc::show_videos(cv::Mat& _img, cv::Mat& _proc_img)
{
    ROS_INFO("SHOW VIDEOS");
    cv::Mat resize_img;
    cv::Mat resize_proc_img;
    if(_img.cols >= 1920 || _img.rows >= 1080)
    {
        cv::Mat resize_img = calculate.resize_img(_img, 0.5);
        cv::Mat resize_proc_img = calculate.resize_img(_proc_img, 0.5);
    }
    else
    {
        resize_img = _img;
        resize_proc_img = _proc_img;
    }
    cv::imshow("Original image", resize_img);
    cv::imshow("Processed image", resize_img);
    cv::moveWindow("Original image", 960, 0);
    cv::moveWindow("Processed image", 960, 540);
    cv::waitKey(1);
}

void Img_Proc::callback(const sensor_msgs::ImageConstPtr& _msg)
{
    ROS_INFO("CALLBACK");
    cv_bridge::CvImagePtr cv_ptr;
    mtx_img.lock();
    try
    {
        cv_ptr = cv_bridge::toCvCopy(_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(const cv::Exception& e)
    {
        return;
    }
    cv::Mat image;
    cv_ptr->image.copyTo(image);   
    mtx_img.unlock();
    cv::Mat proc_img = Preprocessing(image);
    std::vector<cv::Point> contours = find_contour(image, proc_img);
    if(!contours.empty())
    {
        std::vector<Corner> corners = find_corners(image, proc_img, contours);
    }
    else
    {
        ROS_WARN("EMPTY CONTOURS");
    }
    show_videos(image, proc_img);
}

void Img_Proc::subscribe()
{
    img_sub = nh_img.subscribe<sensor_msgs::Image>("/camera/color/image_raw", 10, &Img_Proc::callback, this);
}