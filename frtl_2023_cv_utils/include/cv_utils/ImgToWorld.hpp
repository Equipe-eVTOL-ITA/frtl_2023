// pnp_solver.hpp

#ifndef PNP_SOLVER_HPP
#define PNP_SOLVER_HPP

#include <opencv2/opencv.hpp>

void ImgToWorld(
    const std::vector<cv::Point3f>& objectPoints,
    const std::vector<cv::Point2f>& imagePoints,
    const cv::Mat& cameraMatrix,
    const cv::Mat& distCoeffs,
    cv::Mat& rvec,
    cv::Mat& tvec);

#endif // PNP_SOLVER_HPP
