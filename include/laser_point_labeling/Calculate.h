#include "laser_point_labeling/Corners.h"

class Calculate
{
    private:
    public:
        double get_distance(cv::Point p1, cv::Point p2);
        double get_angle(cv::Point p1, cv::Point p2);
        int get_min_index(std::vector<double> _vector);

        cv::Point translate(cv::Point _ref_pt, cv::Point _moved_pt);
        cv::Point get_centroid(std::vector<cv::Point> p1);

        std::vector<cv::Point> angle_ordering(std::vector<cv::Point>& _corners);
        std::vector<Corner> ordering_corner(std::vector<Corner>& _corners);

    Calculate(){}
    ~Calculate(){}
};

double Calculate::get_distance(cv::Point p1, cv::Point p2)
{
    double dist;
    dist = std::sqrt(std::pow((p1.x-p2.x),2)+std::pow((p1.y-p2.y),2));
    return dist;
}

double Calculate::get_angle(cv::Point p1, cv::Point p2)
{
    return std::atan2(p2.y-p1.y, p2.x-p1.x);
}

int Calculate::get_min_index(std::vector<double> _vector)
{
    return std::min_element(_vector.begin(), _vector.end())-_vector.begin();
}

cv::Point Calculate::translate(cv::Point _ref_pt, cv::Point _moved_pt)
{
    cv::Point trans;
    trans.x = _moved_pt.x-_ref_pt.x;
    trans.y = _moved_pt.y-_ref_pt.y;
    return trans;
}

cv::Point Calculate::get_centroid(std::vector<cv::Point> p1)
{
    cv::Point center(0,0);
    for(int i = 0; i < p1.size(); i++)
    {
        center.x += p1[i].x;
        center.y += p1[i].y;
    }
    center.x /= p1.size();
    center.y /= p1.size();
    return center;
}

std::vector<cv::Point> Calculate::angle_ordering(std::vector<cv::Point>& _corners)
{
    cv::Point centroid(0,0);
    std::vector<double> angle_vec;
    std::vector<cv::Point> ordered_corners;
    for(const auto& corner : _corners)
    {
        centroid.x += corner.x;
        centroid.y += corner.y;
    }
    centroid.x /= _corners.size();
    centroid.y /= _corners.size();

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

bool compare_label(const Corner& _corner_a, const Corner& _corner_b)
{
    return _corner_a.label < _corner_b.label;
}

std::vector<Corner> Calculate::ordering_corner(std::vector<Corner>& _corners)
{
    std::sort(_corners.begin(), _corners.end(), compare_label);

    return _corners;
}