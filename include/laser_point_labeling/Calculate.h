#include "laser_point_labeling/Points.h"

class Calculate
{
    public:
        double get_distance(const cv::Point& _p1, const cv::Point& _p2);
        double get_angle(const cv::Point& _p1, const cv::Point& _p2);
        int get_min_index(const std::vector<double>& _vector);

        cv::Point translate(const cv::Point& _ref_pt, const cv::Point& _moved_pt);
        cv::Point get_centroid(const std::vector<cv::Point>& _p1);

        std::vector<cv::Point> angle_ordering(const std::vector<cv::Point>& _corners);
        std::vector<Corner> label_ordering_corner(std::vector<Corner> _corners);
    Calculate(){}
    ~Calculate(){}
};

double Calculate::get_distance(const cv::Point& _p1, const cv::Point& _p2)
{
    return std::sqrt(std::pow((_p1.x-_p2.x),2)+std::pow((_p1.y-_p2.y),2));
}

double Calculate::get_angle(const cv::Point& _p1, const cv::Point& _p2)
{
    int x_dist = _p2.x - _p1.x;
    int y_dist = _p2.y - _p1.y;
    double angle = std::atan2(y_dist, x_dist);
    return angle;
}

int Calculate::get_min_index(const std::vector<double>& _vector)
{
    return std::min_element(_vector.begin(), _vector.end())-_vector.begin();
}

cv::Point Calculate::translate(const cv::Point& _ref_pt, const cv::Point& _moved_pt)
{
    cv::Point trans;
    trans.x = _moved_pt.x-_ref_pt.x;
    trans.y = _moved_pt.y-_ref_pt.y;
    return trans;
}

cv::Point Calculate::get_centroid(const std::vector<cv::Point>& _p1)
{
    cv::Point centroid(0,0);
    for(int i = 0; i < _p1.size(); i++)
    {
        centroid.x += _p1[i].x;
        centroid.y += _p1[i].y;
    }
    centroid.x /= _p1.size();
    centroid.y /= _p1.size();
    return centroid;
}

std::vector<cv::Point> Calculate::angle_ordering(const std::vector<cv::Point>& _corners)
{
    cv::Point centroid = get_centroid(_corners);
    std::vector<double> angle_vec;
    std::vector<cv::Point> ordered_corners;
    for(const auto& corner : _corners)
    {
        double angle = get_angle(centroid, corner);
        angle_vec.push_back(angle);
    }
    for(int i = 0; i < _corners.size(); i++)
    {
        int min_idx = get_min_index(angle_vec);
        ordered_corners.push_back(_corners[min_idx]);
        angle_vec[min_idx] = INFINITY;
    }
    
    return ordered_corners;
}

bool compare_label(const Corner& _corner_1, const Corner& _corner_2)
{
    return _corner_1.label < _corner_2.label;
}

std::vector<Corner> Calculate::label_ordering_corner(std::vector<Corner> _corners)
{
    std::sort(_corners.begin(), _corners.end(), compare_label);
    return _corners;
}