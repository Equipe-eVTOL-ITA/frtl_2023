#ifndef IMG_TO_WORLD_CONVERTER_HPP
#define IMG_TO_WORLD_CONVERTER_HPP
#include "common.hpp"
#include <fstream>
#include <iostream>
#include <numeric>

typedef std::vector<Eigen::Vector2d> Shape;

class ImageToWorldConverter : public BaseCVUtil<std::vector<cv::Point2i>, Eigen::Vector3d>
{
private:
    const cv::Mat cameraMatrix;
    const Shape shape;
public:
    //helpers for testing
    static cv::Mat readCameraMatrix(std::string camMatFilename);

    ImageToWorldConverter(std::string camMatFilename, Shape shape);

    //posição do drone relativa a marco
    const Eigen::Vector3d apply(std::vector<cv::Point2i> imgPoints);

    cv::Mat displayableResult(const cv::Mat rgbImage, Eigen::Vector3d pos3d);

    void createTrackbars();
};

#endif