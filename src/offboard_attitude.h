#ifndef offboard_attitude_h
#define offboard_attitude_h

#include <Eigen/Dense>

struct TrajectoryPoint{
    float t;
    Eigen::Vector3f pos;
};

#endif