#include <cv_utils/ColorBoxFilter.hpp>

ColorBoxFilter::ColorBoxFilter(uint8_t minH, uint8_t minS,
                uint8_t minV, uint8_t maxH,
                uint8_t maxS, uint8_t maxV)
    :
    minH{minH}, minS{minS}, minV{minV},
    maxH{maxH}, maxS{maxS}, maxV{maxV}
{}

const cv::Mat ColorBoxFilter::apply(cv::Mat rgbImage) {
    cv::Mat blurred;
    cv::GaussianBlur(rgbImage, blurred, {5, 5}, 1.5);
    cv::Mat hsvImage;
    //HSV HAS RANGE 0 - 180 (FITS IN UINT8_T)
    cv::cvtColor(blurred, hsvImage,
            cv::COLOR_BGR2HSV);
    cv::Mat colorRegion;
    cv::inRange(
        hsvImage,
        cv::Mat({minH, minS, minV}),
        cv::Mat({maxH, maxS, maxV}),
        colorRegion);
    return colorRegion;
}

cv::Mat ColorBoxFilter::displayableResult(const cv::Mat rgbImage, cv::Mat colorMask) {
    cv::Mat ret;
    //retorna parte colorida da imagem com a cor original
    cv::bitwise_and(rgbImage, rgbImage, ret, colorMask);
    return ret;
}

#define CBF_TRACKBAR(member, windowName) TRACKBAR(member, windowName, ColorBoxFilter, uint8_t, UINT8_MAX)

void ColorBoxFilter::createTrackbars() {
    std::string windowName = "Box filter trackbars";
    cv::namedWindow(windowName);
    CBF_TRACKBAR(minH, windowName)
    CBF_TRACKBAR(minS, windowName)
    CBF_TRACKBAR(minV, windowName)
    CBF_TRACKBAR(maxH, windowName)
    CBF_TRACKBAR(maxS, windowName)
    CBF_TRACKBAR(maxV, windowName)
}

SAVE_TO_JSON(ColorBoxFilter)

