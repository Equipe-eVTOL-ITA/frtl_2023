#ifndef ELLIPSE_FIT_RESULT_HPP
#define ELLIPSE_FIT_RESULT_HPP
#include "common.hpp"

struct EllipseFitResult {
    const Eigen::Vector3d coefs;  //coefs do fit
    const Eigen::Vector2d centroid;
    const Eigen::Vector2d maxCoords;
    EllipseFitResult(Eigen::Vector3d coefs,
                Eigen::Vector2d centroid, Eigen::Vector2d maxCoords)
        : coefs{coefs}, centroid{centroid}, maxCoords{maxCoords}
    {}
};

#endif