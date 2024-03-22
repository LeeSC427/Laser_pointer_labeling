#include "laser_point_labeling/headers.h"

class Corner
{
    public:
        cv::Point coord;
        int label;

    Corner()
    {}
    ~Corner()
    {}
};


class Process
{
    private:
        Corner corner;
        cv::Scalar low_HSV = cv::Scalar(40, 60, 50);
        cv::Scalar up_HSV = cv::Scalar(80, 255, 255);
    public:
        bool is_init;

        std::vector<Corner> corners_prev;

        cv::Mat Preprocessing(cv::Mat& _img);

        std::vector<cv::Point> find_contour(cv::Mat& _image, cv::Mat& _preprocessed_img);

        std::vector<Corner> find_corners(cv::Mat& _img, cv::Mat& _preprocessed_img, std::vector<cv::Point2f> contour);

        void show_videos(cv::Mat& _img, cv::Mat& _preprocessed_img);

        void orderConers(std::vector<cv::Point2f>& _corners);

        cv::Point2f find_centroid(std::vector<Corner>& _corner_vec);

        cv::Point2f translation(cv::Point2f p1, cv::Point2f p2);

        int find_matching_label(cv::Point2f corner);

        double distance(cv::Point2f p1, cv::Point2f p2);
    Process()
    {
        is_init = true;
    }
    ~Process()
    {}
};

cv::Mat Process::Preprocessing(cv::Mat& _img){
    int img_row_num = _img.size[0];
    int img_col_num = _img.size[1];

    cv::Mat img_hsv;

    cv::cvtColor(_img, img_hsv, cv::COLOR_BGR2HSV);
    cv::inRange(img_hsv, low_HSV, up_HSV, img_hsv);

    return img_hsv;
}

std::vector<cv::Point> Process::find_contour(cv::Mat& _image, cv::Mat& _preprocessed_img){
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(_preprocessed_img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

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

    std::vector<cv::Point> largest_contour = contours[largest_contour_index];

    cv::drawContours(_image, contours, largest_contour_index, cv::Scalar(0,0,127), 2);

    return largest_contour;
}

std::vector<Corner> Process::find_corners(cv::Mat& _img, cv::Mat& _preprocessed_img, std::vector<cv::Point2f> contour){
    double epsilon = 0.1 * cv::arcLength(contour, true);
    std::vector<cv::Point2f> corners;
    cv::TermCriteria term_criteria = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1);

    cv::approxPolyDP(contour, corners, epsilon, true);
    cv::cornerSubPix(_preprocessed_img, corners, cv::Size(11,11), cv::Size(-1,-1), term_criteria);

    orderConers(corners);

    if(corners.size() == 4)
    {
        if(is_init)
        {
            corners_prev.clear();
            for(int j = 0; j < corners.size(); j++)
            {   
                Corner corner;
                cv::circle(_img, corners[j], 3, cv::Scalar(127,0,0), -1);
                cv::putText(_img, std::to_string(j+1), corners[j], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,0), 2);

                corner.coord = corners[j];
                corner.label = j;

                corners_prev.push_back(corner);
            }

            is_init = false;
        }
        else
        {
            cv::Point2f prev_center;
            cv::Point2f cur_center;
            cv::Point2f center_dist;

            if(corner_prev.size() == 4)
            {
                prev_center = find_centroid(corners_prev);
                cur_center = find_centroid(corners);

                center_dist = translation(corners_prev, corners);

                std::vector<cv::Point2f> moved_corners;

                for(int i = 0; i < corners.size(); i++)
                {
                    cv::Point2f moved_corner;
                    moved_corner = corners[i] - center_dist;

                    moved_corners.push_back(moved_corner);
                }

                //Compare distance from moved_corners to prev_corners
                /*Code*/
            }
        }
    }
    else
        is_init = true;
}

void Process::orderConers(std::vector<cv::Point2f>& _corners){
    cv::Point2f centroid(0,0);
    for(const auto& corner : _corners)
        centroid += corner;
    
    centroid *=  (1.0 / _corners.size());

    std::vector<double> angles;
    for(const auto& corner : _corners){
        double angle = std::atan2(corner.y - centroid.y, corner.x - centroid.x);
        angles.push_back(angle);
    }

    std::vector<cv::Point2f> ordered_corners;
    for(int i = 0; i < _corners.size(); ++i)
    {
        int min_idx = std::min_element(angles.begin(), angles.end())-angles.begin();
        ordered_corners.push_back(_corners[min_idx]);
        angles[min_idx] = INFINITY;
    }

    _corners = ordered_corners;
}

cv::Point2f Process::find_centroid(std::vector<Corner>& _corner_vec)
{   
    cv::Point2f center;
    double cent_x = 0.0;
    double cent_y = 0.0;

    for(int i = 0; i < _corner_vec.size(); i++)
    {
        cent_x += _corner_vect[i].coord.x;
        cent_y += _corner_vect[i].coord.y;
    }

    center.x = cent_x /= _corner_vec.size();
    center.y = cent_y /= _corner_vec.size();

    return center;
}

cv::Point2f Process::translation(cv::Point2f p1, cv::Point2f p2)
{
    cv::Point2f trans;

    trans.x = p2.x - p1.x;
    trans.y = p2.y - p1.y;

    return trans;
}

int Process::find_matching_label(cv::Point2f corner)
{
    std::vector<double> dist_vec;

    for(int i = 0; i < corners_prev.size(); i++)
    {
        double dist = distance(corner, corners_prev[i].coord);
        dist_vec.push_back(dist);
    }

    int min_label = std::min_element(dist_vec.begin(), dist_vec.end()) - dist_vec.begin();

    return min_label;
}

double Process::distance(cv::Point2f p1, cv::Point2f p2)
{
    return std::sqrt(std::pow((p1.x - p2.x),2) + std::pow((p1.y - p2.y),2));
}