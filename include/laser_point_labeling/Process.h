#include "laser_point_labeling/Calculate.h"

class Process
{
    private:
        cv::Scalar low_HSV = cv::Scalar(40,50,50);
        cv::Scalar up_HSV = cv::Scalar(80,255,255);
        Calculate cal;
    public:
        ros::NodeHandle nh_img;
        ros::Subscriber sub_img;
        std::mutex mtx_img;
        std::vector<Corner> prev_corner;

        void subscribe();
        void loop(const sensor_msgs::ImageConstPtr& _msg);

        cv::Mat Preprocess(cv::Mat& _img);

        std::vector<cv::Point> Find_contour(cv::Mat& _img, cv::Mat& _proc_img);
        std::vector<cv::Point> Find_corner(cv::Mat& _proc_img, std::vector<cv::Point>& _contour);

        std::vector<Corner> find_first_corner(cv::Mat& _proc_img, std::vector<cv::Point>& _corner_vec);
        std::vector<Corner> find_matching_corner(cv::Mat& _img, cv::Mat& _proc_img, std::vector<cv::Point> _corners);
    
        void Draw_corner(cv::Mat& _img, cv::Mat& _proc_img, std::vector<Corner>& _corner_vec);
        void Show_video(cv::Mat& _img, cv::Mat& _proc_img);
    Process(){}
    ~Process(){}
};

void Process::subscribe()
{
    sub_img = nh_img.subscribe<sensor_msgs::Image>("/camera/color/image_raw", 10, &Process::loop, this);
}

void Process::loop(const sensor_msgs::ImageConstPtr& _msg)
{
    mtx_img.lock();
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
        return;
    }
    cv::Mat img;
    cv_ptr->image.copyTo(img);
    mtx_img.unlock();
}

cv::Mat Process::Preprocess(cv::Mat& _img)
{
    cv::Mat proc_img;
    cv::cvtColor(_img, proc_img, cv::COLOR_BGR2HSV);
    cv::inRange(proc_img, low_HSV, up_HSV, proc_img);
    return proc_img;
}

std::vector<cv::Point> Process::Find_contour(cv::Mat& _img, cv::Mat& _proc_img)
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(_proc_img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    int largest_contour_index = 0;
    double largest_area = 0;
    std::vector<cv::Point> largest_contour;
    largest_contour.clear();

    if(!contours.empty())
    {
        for(int i = 0; i < contours.size(); i++)
        {
            double area = cv::contourArea(contours[i]);
            if(area > largest_area)
            {
                largest_area = area;
                largest_contour_index = i;
            }
        }
        cv::drawContours(_img, contours, largest_contour_index, cv::Scalar(0,255,255), 3);
        cv::drawContours(_proc_img, contours, largest_contour_index, cv::Scalar(0,255,255), 3);
        largest_contour = contours[largest_contour_index];
    }
    return largest_contour;
}

std::vector<cv::Point> Process::Find_corner(cv::Mat& _proc_img, std::vector<cv::Point>& _contour)
{
    double epsilon = 0.1 * cv::arcLength(_contour, true);
    std::vector<cv::Point2f> double_corners;
    cv::TermCriteria termcriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 30, 0.1);
    cv::approxPolyDP(_contour, double_corners, epsilon, true);
    cv::cornerSubPix(_proc_img, double_corners, cv::Size(11,11), cv::Size(-1,-1), termcriteria);
    std::vector<cv::Point> corners;
    for(int i = 0; i < double_corners.size(); i++)
    {
        cv::Point _corner;
        _corner.x = (int)double_corners[i].x;
        _corner.y = (int)double_corners[i].y;
        corners.push_back(_corner);
    }
    return corners;
}