#ifndef CONTOUR_EXTRACTOR_HPP
#define CONTOUR_EXTRACTOR_HPP
#include "common.hpp"

class ContourExtractor : public BaseCVUtil<cv::Mat, ContourList>
{
private:
    unsigned int sizeThreshold;
public:
    ContourExtractor(unsigned int sizeThreshold);

    const ContourList apply(cv::Mat region);

    cv::Mat displayableResult(const cv::Mat rgbImage, ContourList contours);

    void createTrackbars();

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(ContourExtractor, sizeThreshold)

    void save_to_json(std::string filename);

};

#endif