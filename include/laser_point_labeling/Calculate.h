#include "laser_point_labeling/headers.h"
#include "laser_point_labeling/Points.h"

class Calculate
{
    public:

    cv::Point2f find_centroid(std::vector<Corner>& _corner_vec);
    cv::Point2f find_centroid(std::vector<cv::Point2f>& _corner_vec);

    cv::Point2f translation(cv::Point2f p1, cv::Point2f p2);
    std::vector<cv::Point2f> move_points(cv::Point2f _dist, std::vector<cv::Point2f> _corners);

    double distance(cv::Point2f p1, cv::Point2f p2);
    std::vector<double> angles(std::vector<cv::Point2f>& _corners_vec, cv::Point2f center);

    std::vector<cv::Point2f> ordering(std::vector<cv::Point2f>& _corners);

    Calculate(){}
    ~Calculate(){}
};

cv::Point2f Calculate::find_centroid(std::vector<Corner>& _corner_vec)
{
    ROS_INFO("FIND CENTROID");

    cv::Point2f center;
    double cen_x(0.0), cen_y(0.0);

    for(int i = 0; i < _corner_vec.size(); i++)
    {
        cen_x += _corner_vec[i].coord.x;
        cen_y += _corner_vec[i].coord.y;
    }

    center.x = cen_x / _corner_vec.size();
    center.y = cen_y / _corner_vec.size();

    return center;
}

cv::Point2f Calculate::find_centroid(std::vector<cv::Point2f>& _corner_vec)
{
    ROS_INFO("FIND CENTROID");

    cv::Point2f center;
    double cen_x(0.0), cen_y(0.0);

    for(int i = 0; i < _corner_vec.size(); i++)
    {
        cen_x += _corner_vec[i].x;
        cen_y += _corner_vec[i].y;
    }

    center.x = cen_x / _corner_vec.size();
    center.y = cen_y / _corner_vec.size();

    return center;
}

cv::Point2f Calculate::translation(cv::Point2f p1, cv::Point2f p2)
{
    ROS_INFO("TRANSLATION");

    cv::Point2f trans;

    trans.x = p2.x - p1.x;
    trans.y = p2.y - p1.y;

    return trans;
}

std::vector<cv::Point2f> Calculate::move_points(cv::Point2f _dist, std::vector<cv::Point2f> _corners)
{
    ROS_INFO("MOVE POINTS");

    std::vector<cv::Point2f> moved_points;
    moved_points.clear();

    for(int i = 0; i < _corners.size(); i++)
    {
        cv::Point2f moved_point;
        moved_point = _corners[i] - _dist;
        moved_points.push_back(moved_point);
    }

    return moved_points;
}

double Calculate::distance(cv::Point2f p1, cv::Point2f p2)
{
    return std::sqrt(std::pow((p1.x-p2.x),2) + std::pow((p1.y-p2.y),2));
}

std::vector<double> Calculate::angles(std::vector<cv::Point2f>& _corners_vec, cv::Point2f center)
{
    ROS_INFO("ANGLES");

    std::vector<double> angle_vec;
    for(const auto& corner : _corners_vec)
    {
        double angle = std::atan2(corner.y-center.y, corner.x-center.x);
        angle_vec.push_back(angle);
    }
    return angle_vec;
}

std::vector<cv::Point2f> Calculate::ordering(std::vector<cv::Point2f>& _corners_vec)
{
    ROS_INFO("ORDERING");

    cv::Point2f centroid;
    std::vector<double> angle_vec;
    std::vector<cv::Point2f> ordered_corners;

    centroid = find_centroid(_corners_vec);
    angle_vec = angles(_corners_vec, centroid);

    for(int i = 0; i < _corners_vec.size(); ++i)
    {
        int min_idx = std::min_element(angle_vec.begin(), angle_vec.end())-angle_vec.begin();
        ordered_corners.push_back(_corners_vec[min_idx]);
        angle_vec[min_idx] = INFINITY;
    }

    return ordered_corners;
}