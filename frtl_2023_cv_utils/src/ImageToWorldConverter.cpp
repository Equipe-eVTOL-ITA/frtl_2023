#include <cv_utils/ImageToWorldConverter.hpp>



cv::Mat ImageToWorldConverter::readCameraMatrix(std::string camMatFilename) {
    std::ifstream camMatFile(camMatFilename);
    cv::Mat cameraMatrix(3, 3, CV_64F);
    for (int i = 0; i < 9; i++) {
        double f;
        camMatFile >> f;
        cameraMatrix.at<double>(i/3, i%3) = f;
    }
    return cameraMatrix;
}

ImageToWorldConverter::ImageToWorldConverter(std::string camMatFilename, Shape shape) 
    : cameraMatrix{readCameraMatrix(camMatFilename)}, shape{shape}
{}

const Eigen::Vector3d ImageToWorldConverter::apply(std::vector<cv::Point2i> imgPoints) {
    imgPoints.at(0);
    return {0,0,0};
}


cv::Mat ImageToWorldConverter::displayableResult(const cv::Mat rgbImage, Eigen::Vector3d pos3d) {
    //https://stackoverflow.com/questions/37508017/how-to-convert-eigen-librarys-matrix-or-vector-types-to-string
    std::stringstream pointNameStream;
    pointNameStream << pos3d;
    std::string pointName = pointNameStream.str();
    
    Eigen::Vector2d avgPos = {0.0, 0.0};
    cv::Point avgPoint(avgPos[0], avgPos[1]);
    
    cv::Mat ret = rgbImage.clone();
    cv::putText(ret, pointName, avgPoint, cv::FONT_HERSHEY_SIMPLEX, 14, 0);
    return ret;
}

void ImageToWorldConverter::createTrackbars() {

}