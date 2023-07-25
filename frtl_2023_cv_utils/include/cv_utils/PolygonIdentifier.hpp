#ifndef POLYGON_IDENTIFIER_HPP
#define POLYGON_IDENTIFIER_HPP
#include "common.hpp"

#include "PolyIdResult.hpp"

class PolygonIdentifier : public BaseCVUtil<ContourList, std::vector<PolyIdResult>>
{
private:
    unsigned int vertexNumber, vertexNumberTolerance;
    bool isConvex;
    float maxArea_PerimeterSquared;

    bool vertexCriterion(std::vector<cv::Point2i> polyVerts);
    bool concavityCriterion(std::vector<cv::Point2i> polyVerts);
    bool area_PerimSqrCriterion(double area, double perimeter);

public:
    PolygonIdentifier(unsigned int vertexNumber, unsigned int vertexNumberTolerance,
                    bool isConvex, float maxArea_PerimeterSquared);
    //trocar vectors pelo melhor ajuste?
    const std::vector<PolyIdResult> apply(ContourList contours);

    cv::Mat displayableResult(const cv::Mat rgbImg, std::vector<PolyIdResult> polyResults);

    void createTrackbars();

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(PolygonIdentifier, vertexNumber, vertexNumberTolerance, isConvex, maxArea_PerimeterSquared)

    void save_to_json(std::string filename);

};


#endif