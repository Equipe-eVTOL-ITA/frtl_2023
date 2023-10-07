#ifndef BASEDETECTION_CPP
#define BASEDETECTION_CPP

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <Eigen/Core>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

class BaseDetection
{
private:
        /* data */
public:
        BaseDetection();

        std::vector<cv::Point2f> ellipseDetection(cv::Mat img);
        std::vector<std::vector<cv::Point>> getContours(cv::Mat image);
        cv::Point2f getContourCenter(std::vector<cv::Point> contour);
        std::vector<cv::Point2f> getKNearest(const std::vector<std::vector<cv::Point2f>>& poly, const cv::Point2f& point); //k=4
        std::vector<std::pair<cv::Point2f, std::vector<cv::Point2f>>> crossDetection(cv::Mat image);
        std::vector<std::pair<cv::Point2f, std::vector<cv::Point2f>>> landingPadDetection(cv::Mat image);
};


#endif