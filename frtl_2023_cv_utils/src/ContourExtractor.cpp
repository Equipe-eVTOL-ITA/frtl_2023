#include <cv_utils/ContourExtractor.hpp>
#include <algorithm>
ContourExtractor::ContourExtractor(unsigned int sizeThreshold)
    : sizeThreshold{sizeThreshold}
{}

const ContourList ContourExtractor::apply(cv::Mat region) {
    ContourList allContours;
    cv::findContours(region, allContours,
        cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
    ContourList output;
    std::copy_if(allContours.begin(), allContours.end(), 
                std::back_inserter(output),
                 [this](std::vector<cv::Point2i> contour) {
                     return contour.size() >= this->sizeThreshold;
                 });
    return output;
}

cv::Mat ContourExtractor::displayableResult(const cv::Mat rgbImage, ContourList contours) {
    
    cv::Mat displayImage = rgbImage.clone();
    cv::drawContours(displayImage, contours, -1, {0,0,0}, 5);

    return displayImage;
}

void ContourExtractor::createTrackbars() {
    std::string windowName = "Contour extractor trackbars";
    cv::namedWindow(windowName);
    //depends on int being 64 bits
    TRACKBAR(sizeThreshold, windowName, ContourExtractor, unsigned int, 200);
}

SAVE_TO_JSON(ContourExtractor)