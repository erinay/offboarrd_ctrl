#ifndef offboard_attitude_h
#define offboard_attitude_h

#include <Eigen/Dense>

struct TrajectoryPoint{
    float t;
    Eigen::Vector3f pos;
    Eigen::Vector3f vel;
    Eigen::Vector3f acc;
};

#endif