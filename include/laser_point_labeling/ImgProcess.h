#include "laser_point_labeling/Points.h"
#include "laser_point_labeling/Calculate.h"

class ImgProcess
{
    private:
        Calculate calculate;

        cv::Scalar low_HSV = cv::Scalar(40,60,50);
        cv::Scalar up_HSV = cv::Scalar(80,255,255);

        std::mutex mtx_img;
    public:
        ros::NodeHandle nh_img;
        ros::Subscriber img_sub;

        bool is_init;

        std::vector<Corner> prev_corners;

        std::vector<cv::Point> find_contour(cv::Mat& _img, cv::Mat& _proc_img);
        std::vector<Corner> find_corners(cv::Mat& _img, cv::Mat& _proc_img, std::vector<cv::Point> _contours);
        std::vector<Corner> find_first_corners(std::vector<cv::Point2f>& _corners);
        std::vector<Corner> find_recur_corners(std::vector<cv::Point2f>& _corners);

        std::vector<Corner> matching_point(std::vector<cv::Point2f> _corners, std::vector<cv::Point2f> _moved_corners);

        cv::Mat Preprocessing(cv::Mat& _img);

        cv::Mat resize_image(cv::Mat& _img, double _scale);
        void draw_corners(cv::Mat& _img, cv::Mat& _proc_img, std::vector<Corner> _cur_corners);
        void show_videos(cv::Mat& _img, cv::Mat& _proc_img);

        void subscribe();
        void Callback(const sensor_msgs::ImageConstPtr& _msg);

    ImgProcess()
    {
        is_init = true;
        prev_corners.clear();
    }
    ~ImgProcess(){}
};

std::vector<cv::Point> ImgProcess::find_contour(cv::Mat& _img, cv::Mat& _proc_img)
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
                largest_contour_index = i;
            }
        }

        largest_contour = contours[largest_contour_index];

        cv::drawContours(_img, contours, largest_contour_index, cv::Scalar(0,0,127), 2);

        return largest_contour;
    }
}

std::vector<Corner> ImgProcess::find_corners(cv::Mat& _img, cv::Mat& _proc_img, std::vector<cv::Point> _contours)
{
    ROS_INFO("FIND CORNERS");

    std::vector<Corner> cur_corners;
    std::vector<cv::Point2f> corners;
    double epsilon = 0.1 * cv::arcLength(_contours, true);
    cv::TermCriteria term_criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1);

    cv::approxPolyDP(_contours, corners, epsilon, true);
    cv::cornerSubPix(_proc_img, corners, cv::Size(11,11), cv::Size(-1,-1), term_criteria);

    if(corners.size() == 4)
    {
        if(is_init)
            cur_corners = find_first_corners(corners);
        else
            cur_corners = find_recur_corners(corners);
        
        if(cur_corners.size() == 4)
        {
            prev_corners = cur_corners;
        }
        else
        {
            prev_corners.clear();
        }

        draw_corners(_img, _proc_img, cur_corners);
    }
    else
    {
        prev_corners.clear();
        is_init = true;
        ROS_WARN("NUMBER OF PREVIOUS CORNERS ARE NOT 4");
    }
    
    return cur_corners;
}

std::vector<Corner> ImgProcess::find_first_corners(std::vector<cv::Point2f>& _corners)
{
    ROS_INFO("FIND FIRST CORNERS");

    Corner corner;
    std::vector<Corner> temp_corners;
    calculate.ordering(_corners);
    prev_corners.clear();

    for(int i = 0; i < _corners.size(); i++)
    {
        corner.coord = _corners[i];
        corner.label = i;

        temp_corners.push_back(corner);
    }

    is_init = false;

    return temp_corners;
}

std::vector<Corner> ImgProcess::find_recur_corners(std::vector<cv::Point2f>& _corners)
{
    ROS_INFO("FIND RECUR CORNERS");

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
        is_init = true;
    }

    return temp_corners;
}

std::vector<Corner> ImgProcess::matching_point(std::vector<cv::Point2f> _corners, std::vector<cv::Point2f> _moved_corners)
{
    ROS_INFO("MATCHING POINT");

    std::vector<Corner> temp_corners;
    Corner corner;
    for(int i = 0; i < _moved_corners.size(); i++)
    {
        double min_dist = INFINITY;
        int min_index;
        for(int j = 0; j < prev_corners.size(); j++)
        {
            double dist = calculate.distance(_moved_corners[i], prev_corners[j].coord);

            if(min_dist > dist)
            {
                min_dist = dist;
                min_index = j;
            }
        }

        corner.coord = _corners[min_index];
        corner.label = min_index;
        temp_corners.push_back(corner);
    }

    return temp_corners;
}

cv::Mat ImgProcess::Preprocessing(cv::Mat& _img)
{
    ROS_INFO("PREPROCESSING");

    int img_row_num = _img.rows;
    int img_col_num = _img.cols;

    cv::Mat img_hsv;

    cv::cvtColor(_img, img_hsv, cv::COLOR_BGR2HSV);
    cv::inRange(img_hsv, low_HSV, up_HSV, img_hsv);

    return img_hsv;
}

cv::Mat ImgProcess::resize_image(cv::Mat& _img, double _scale)
{
    ROS_INFO("RESIZE IMAGE");

    cv::Mat resized_img;
    cv::resize(_img, resized_img, cv::Size(_img.cols/_scale, _img.rows/_scale));
    return resized_img;
}

void ImgProcess::draw_corners(cv::Mat& _img, cv::Mat& _proc_img, std::vector<Corner> _cur_corners)
{
    ROS_INFO("DRAW CORNERS");

    for(int i = 0; i < _cur_corners.size(); i++)
    {
        cv::circle(_img, _cur_corners[i].coord, 3, cv::Scalar(127,0,0), -1);
        cv::putText(_img, std::to_string(_cur_corners[i].label), _cur_corners[i].coord, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,0), 2);
        cv::circle(_proc_img, _cur_corners[i].coord, 3, cv::Scalar(127,0,0), -1);
        cv::putText(_proc_img, std::to_string(_cur_corners[i].label), _cur_corners[i].coord, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,255,0), 2);
    }
}

void ImgProcess::show_videos(cv::Mat& _img, cv::Mat& _proc_img)
{
    ROS_INFO("SHOW VIDEOS");

    cv::Mat resize_img;
    cv::Mat resize_proc_img;

    if(_img.cols >= 1920 || _img.rows >= 1080)
    {
        cv::Mat resize_img = resize_image(_img, 2.0);
        cv::Mat resize_proc_img = resize_image(_proc_img, 2.0);
    }
    else
    {
        resize_img = _img;
        resize_proc_img = _proc_img;
    }

    cv::imshow("Original image", resize_img);
    cv::imshow("Processed image", resize_proc_img);
    cv::moveWindow("Original image", 960, 0);
    cv::moveWindow("Processed image", 960, 600);
    cv::waitKey(1);
}

void ImgProcess::Callback(const sensor_msgs::ImageConstPtr& _msg)
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
        ROS_WARN("EMPTY CONTOURS");

    show_videos(image, proc_img);
}

void ImgProcess::subscribe()
{
    img_sub = nh_img.subscribe<sensor_msgs::Image>("/camera/color/image_raw", 10, &ImgProcess::Callback, this);
}