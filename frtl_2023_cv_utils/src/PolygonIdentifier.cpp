#include <cv_utils/PolygonIdentifier.hpp>

bool PolygonIdentifier::vertexCriterion(std::vector<cv::Point2i> polyVerts) {
    return labs(polyVerts.size() - vertexNumber) < vertexNumberTolerance;
}

bool PolygonIdentifier::concavityCriterion(std::vector<cv::Point2i> polyVerts) {
    //no python testava a convexidade do contorno (ruim) 
    return (cv::isContourConvex(polyVerts) == isConvex);
}

bool PolygonIdentifier::area_PerimSqrCriterion(double area, double perimeter) {
    if (perimeter > 1e-3)
        return (area / (perimeter*perimeter) < maxArea_PerimeterSquared);
    else
        return false;
}

PolygonIdentifier::PolygonIdentifier(unsigned int vertexNumber, unsigned int vertexNumberTolerance,
                    bool isConvex, float maxArea_PerimeterSquared)
    : vertexNumber{vertexNumber}, vertexNumberTolerance{vertexNumberTolerance},
        isConvex{isConvex}, maxArea_PerimeterSquared{maxArea_PerimeterSquared}
{}

const std::vector<PolyIdResult> PolygonIdentifier::apply(ContourList contours) {
    std::vector<PolyIdResult> results;
    
    for (auto contourIt = contours.begin(); contourIt < contours.end(); contourIt++) {
    
        cv::Moments M = cv::moments(*contourIt);
        double area = M.m00;
        double perimeter = cv::arcLength(*contourIt, true);
        
        std::vector<cv::Point2i> polygonVertexes;
        cv::approxPolyDP(*contourIt, polygonVertexes, 0.01*perimeter, true);
        //criterios de identificação
        if (vertexCriterion(polygonVertexes)
                && concavityCriterion(polygonVertexes)
                && area_PerimSqrCriterion(area, perimeter)
                && area > 1e-5) { //evita divisão por 0
            results.push_back(PolyIdResult(M.m10/area, M.m01/area, polygonVertexes));
        }
    }
    return results;
}

cv::Mat PolygonIdentifier::displayableResult(const cv::Mat rgbImg, std::vector<PolyIdResult> polyResults) {
    cv::Mat ret = rgbImg.clone();
    for (auto resIt = polyResults.begin(); resIt < polyResults.end(); resIt++)
        cv::polylines(ret, resIt->vertexes, true, {0,0,0}, 5);
    return ret;
}

void PolygonIdentifier::createTrackbars() {
    std::string windowName = "Polygon Identifier trackbars";
    cv::namedWindow(windowName);
    TRACKBAR(vertexNumber, windowName, PolygonIdentifier, unsigned int, 20)
    TRACKBAR(vertexNumberTolerance, windowName, PolygonIdentifier, unsigned int, 10)
    TRACKBAR(isConvex, windowName, PolygonIdentifier, bool, 1)
    cv::createTrackbar("maxArea_PerimeterSquared", windowName, NULL, 1000, \
    [](int val, void* instance) { \
            ((PolygonIdentifier*)instance)->maxArea_PerimeterSquared = 0.08 * (float)val / 1000.0f; \
    }, this);
}

SAVE_TO_JSON(PolygonIdentifier)