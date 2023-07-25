#ifndef COLOR_SEGMENTATION_HPP
#define COLOR_SEGMENTATION_HPP

#include "common.hpp"

// filtro de cor por caixa HSV
class ColorBoxFilter : public BaseCVUtil<cv::Mat, cv::Mat>
{
private:
    uint8_t minH, minS, minV;
    uint8_t maxH, maxS, maxV;

public:
    ColorBoxFilter() {}

    ColorBoxFilter(uint8_t minH, uint8_t minS,
                   uint8_t minV, uint8_t maxH,
                   uint8_t maxS, uint8_t maxV);

    const cv::Mat apply(cv::Mat rgbImage);

    cv::Mat displayableResult(const cv::Mat rgbImage, cv::Mat colorMask);

    void createTrackbars();

    // define conversão de e para json
    //(https://github.com/nlohmann/json)
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(ColorBoxFilter, minH, minS, minV, maxH, maxS, maxV)

    void save_to_json(std::string filename);
    bool load_from_json(std::string filename);
};

#endif