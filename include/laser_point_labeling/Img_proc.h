#include "laser_point_labeling/Calculate.h"

class Img_proc
{
    private:
        cv::Scalar low_HSV = cv::Scalar(40,50,50);
        cv::Scalar up_HSV = cv::Scalar(80,255,255);
        Calculate cal;
    public:
        bool initial_corner;
        ros::NodeHandle nh_img;
        ros::Subscriber sub_img;
        std::mutex mtx_img;
        std::vector<Corner> prev_corner;

        void subscribe();
        void loop(const sensor_msgs::ImageConstPtr& _msg);
        void draw_corner(cv::Mat& _img, cv::Mat& _proc_img, std::vector<Corner> _corner_vec);
        void show_video(cv::Mat& _img, cv::Mat& _proc_img);

        cv::Mat preprocess(cv::Mat& _img);

        std::vector<cv::Point> find_contour(cv::Mat& _img, cv::Mat& _proc_img);
        std::vector<cv::Point> find_corner(cv::Mat& _proc_img, std::vector<cv::Point> _contour);

        std::vector<Corner> find_first_corner(cv::Mat& _proc_img, std::vector<cv::Point> _corners);
        std::vector<Corner> find_matching_corner(cv::Mat& _img, cv::Mat& _proc_img, std::vector<cv::Point> _corners);

    Img_proc():initial_corner(true){}
    ~Img_proc(){}
};

void Img_proc::subscribe()
{
    sub_img = nh_img.subscribe<sensor_msgs::Image>("/camera/color/image_raw", 10, &Img_proc::loop, this);
}

void Img_proc::loop(const sensor_msgs::ImageConstPtr& _msg)
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
    cv::Mat img_mat;
    cv_ptr->image.copyTo(img_mat);
    mtx_img.unlock();

    cv::Mat proc_img = preprocess(img_mat);
    std::vector<cv::Point> contour = find_contour(img_mat, proc_img);
    std::vector<Corner> ordered_corner;

    if(!contour.empty())
    {
        std::vector<cv::Point> corner = find_corner(proc_img, contour);
        if(initial_corner)
        {
            ordered_corner = find_first_corner(proc_img, corner);
            initial_corner = false;
            prev_corner = ordered_corner;
            draw_corner(img_mat, proc_img, ordered_corner);
        }
        else
        {
            if(prev_corner.size() == 4)
            {
                ordered_corner = find_matching_corner(img_mat, proc_img, corner);
                prev_corner = ordered_corner;
                draw_corner(img_mat, proc_img, ordered_corner);
            }
            else
            {
                ROS_WARN("NUMBER OF PREVIOUS CORNERS ARE NOT 4");
                prev_corner.clear();
                initial_corner = true;
            }
        }
    }
    else
    {
        prev_corner.clear();
        initial_corner = true;
    }

    show_video(img_mat, proc_img);
}

cv::Mat Img_proc::preprocess(cv::Mat& _img)
{
    //ROS_INFO("PREPROCESS");
    cv::Mat proc_img;
    cv::cvtColor(_img, proc_img, cv::COLOR_BGR2HSV);
    cv::inRange(proc_img, low_HSV, up_HSV, proc_img);
    return proc_img;
}

std::vector<cv::Point> Img_proc::find_contour(cv::Mat& _img, cv::Mat& _proc_img)
{
    //ROS_INFO("FIND CONTOUR");
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
        cv::drawContours(_proc_img, contours, largest_contour_index, cv::Scalar(0,127,127), 3);
        largest_contour = contours[largest_contour_index];
    }

    return largest_contour;
}

std::vector<cv::Point> Img_proc::find_corner(cv::Mat& _proc_img, std::vector<cv::Point> _contour)
{
    //ROS_INFO("FIND CORNER");
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

std::vector<Corner> Img_proc::find_first_corner(cv::Mat& _proc_img, std::vector<cv::Point> _corners)
{
    //ROS_INFO("FIND FIRST CORNER");
    std::vector<cv::Point> ordered_vec;
    std::vector<Corner> ordered_corner;

    ordered_vec = cal.angle_ordering(_corners);
    for(int i = 0; i < ordered_vec.size(); i++)
    {
        Corner temp_corner;
        temp_corner.coord = ordered_vec[i];
        temp_corner.label = i;
        ordered_corner.push_back(temp_corner);
    }
    
    return ordered_corner;
}

std::vector<Corner> Img_proc::find_matching_corner(cv::Mat& _img, cv::Mat& _proc_img, std::vector<cv::Point> _corners)
{
    ROS_INFO("FIND MATCHING CORNER");
    std::vector<Corner> ordered_corner;
    std::vector<cv::Point> prev_corner_coord;
    for(int i = 0; i < prev_corner.size(); i++)
    {
        cv::Point temp_coord;
        temp_coord = prev_corner[i].coord;
        prev_corner_coord.push_back(temp_coord);
    }
    cv::Point prev_center = cal.get_centroid(prev_corner_coord);
    cv::Point corner_center = cal.get_centroid(_corners);
    cv::Point translate = cal.translate(prev_center, corner_center);
    std::vector<cv::Point> moved_corners;

    for(int i = 0; i < _corners.size(); i++)
    {
        cv::Point temp_coord;
        temp_coord.x = _corners[i].x - translate.x;
        temp_coord.y = _corners[i].y - translate.y;
        moved_corners.push_back(temp_coord);
    }
    cv::line(_img, moved_corners[0], moved_corners[1], cv::Scalar(0,0,0), 2);
    cv::line(_img, moved_corners[1], moved_corners[2], cv::Scalar(0,0,0), 2);
    cv::line(_img, moved_corners[2], moved_corners[3], cv::Scalar(0,0,0), 2);
    cv::line(_img, moved_corners[3], moved_corners[0], cv::Scalar(0,0,0), 2);
    for(int i = 0; i < moved_corners.size(); i++)
    {
        bool error_flag = false;
        Corner temp_corner;
        std::vector<double> dist_vec;

        for(int j = 0; j < prev_corner_coord.size(); j++)
        {
            double dist = cal.get_distance(moved_corners[i], prev_corner_coord[j]);
            dist_vec.push_back(dist);
            if(i == 0)
            {
                if(std::abs(moved_corners[i].x-prev_corner_coord[j].x) >= 30)
                {
                    ROS_WARN("translate: (%d, %d)", translate.x, translate.y);
                    ROS_WARN("CENTER: (%d, %d)", _img.rows/2, _img.cols/2);
                    ROS_WARN("moved_corner's x changes too fast");
                    ROS_INFO("Corner %d's coordinate: (%d, %d)", i, moved_corners[i].x, moved_corners[i].y);
                    ROS_INFO("Corner %d's coordinate: (%d, %d)", i+1, moved_corners[i+1].x, moved_corners[i+1].y);
                    ROS_INFO("Corner %d's coordinate: (%d, %d)", i+2, moved_corners[i+2].x, moved_corners[i+2].y);
                    ROS_INFO("Corner %d's coordinate: (%d, %d)", i+3, moved_corners[i+3].x, moved_corners[i+3].y);
                    cv::circle(_img, moved_corners[i], 10, cv::Scalar(255,255,0), -1);
                }
                ROS_INFO("Prev_corner %d's coordinate: (%d, %d)", j, prev_corner_coord[j].x, prev_corner_coord[j].y);
                ROS_INFO("Corner %d to prev_corner %d --> %lf", i, j, dist);
            }
        }
        int min_idx = cal.get_min_index(dist_vec);
        ROS_INFO("Min dist is from corner %d to prev_corner %d --> %lf", i, min_idx, dist_vec[min_idx]);
        temp_corner.coord = _corners[min_idx];
        temp_corner.label = min_idx;
        prev_corner_coord[min_idx] = cv::Point(INT_MAX, INT_MAX);
        ordered_corner.push_back(temp_corner);
    }
    ordered_corner = cal.ordering_corner(ordered_corner);

    return ordered_corner;
}

void Img_proc::draw_corner(cv::Mat& _img, cv::Mat& _proc_img, std::vector<Corner> _corner_vec)
{
    //ROS_INFO("DRAW CORNER");
    for(int i = 0; i < _corner_vec.size(); i++)
    {
        cv::circle(_img, _corner_vec[i].coord, 4, cv::Scalar(0,0,255), -1);
        cv::putText(_img, std::to_string(_corner_vec[i].label), _corner_vec[i].coord, cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255,0,0), 3);
    }
    for(int i = 0; i < prev_corner.size(); i++)
    {
        cv::circle(_img, prev_corner[i].coord, 2, cv::Scalar(255,0,0), -1);
        cv::putText(_img, std::to_string(prev_corner[i].label), prev_corner[i].coord, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,255), 2);
    }
}

void Img_proc::show_video(cv::Mat& _img, cv::Mat& _proc_img)
{
    //ROS_INFO("SHOW VIDEOS");

    cv::Mat resize_img;
    cv::Mat resize_proc_img;

    if(_img.cols >= 1920 || _img.rows >= 1080)
    {
        cv::resize(_img, resize_img, cv::Size(_img.size[1] / 2, _img.size[0] / 2));
        cv::resize(_proc_img, resize_proc_img, cv::Size(_proc_img.size[1] / 2, _proc_img.size[0] / 2));
    }
    else
    {
        resize_img = _img;
        resize_proc_img = _proc_img;
    }

    cv::imshow("Original image", resize_img);
    cv::imshow("Filtered image", resize_proc_img);
    cv::moveWindow("Original image", 960, 0);
    cv::moveWindow("Filtered image", 960, 640);
    cv::waitKey(1);
}