#include "laser_point_labeling/headers.h"

#ifndef Corner_H
#define Corner_H

class Corner
{
    public:
        cv::Point coord;
        int label;
    Corner(){}
    ~Corner(){}
};

#endif