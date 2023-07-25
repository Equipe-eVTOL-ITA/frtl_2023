#ifndef POLY_ID_RESULT_HPP
#define POLY_ID_RESULT_HPP
#include "common.hpp"

struct PolyIdResult {
    Eigen::Vector2d center;
    std::vector<cv::Point2i> vertexes;
    PolyIdResult(double cX, double cY,
            std::vector<cv::Point2i> vertexes)
        : center{cX, cY}, vertexes{vertexes}
    {}
};

#endif