#include <cv_utils/BaseDetector.hpp>

std::vector<cv::Point2f> BaseDetection::ellipseDetection(cv::Mat img) {
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    cv::GaussianBlur(gray, gray, cv::Size(7, 7), 20);

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, 30, 130, 70, 3, 0);

    std::vector<cv::Point2f> centers;

    if (!circles.empty()) {
        for (size_t i = 0; i < circles.size(); i++) {
            cv::Point2f center(circles[i][0], circles[i][1]);
            centers.push_back(center);
        }
    }

    return centers;
}

std::vector<std::vector<cv::Point>> BaseDetection::getContours(cv::Mat image) {
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    cv::GaussianBlur(gray, gray, cv::Size(7, 7), 20);

    cv::Mat edged;
    cv::Canny(gray, edged, 40, 100, true);

    cv::Mat structuringElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::Mat dilated;
    cv::morphologyEx(edged, dilated, cv::MORPH_DILATE, structuringElement);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(dilated, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    return contours;
}

cv::Point2f BaseDetection::getContourCenter(std::vector<cv::Point> contour) {
    cv::Moments M = cv::moments(contour);
    cv::Point2f center(M.m10 / M.m00, M.m01 / M.m00);
    return center;
}


std::vector<cv::Point2f> BaseDetection::getKNearest(const std::vector<std::vector<cv::Point2f>>& poly, const cv::Point2f& point) {
    float x = point.x;
    float y = point.y;
    int k = 4;

    // Define a custom lambda function for sorting by distance from (x, y)
    auto distanceComparator = [x, y](const std::vector<cv::Point2f>& pt1, const std::vector<cv::Point2f>& pt2) {
        float dist1 = std::pow(pt1[0].x - x, 2) + std::pow(pt1[0].y - y, 2);
        float dist2 = std::pow(pt2[0].x - x, 2) + std::pow(pt2[0].y - y, 2);
        return dist1 < dist2;
    };

    // Sort the poly vector by distance using the custom lambda function
    std::vector<std::vector<cv::Point2f>> sortedPoly = poly;
    std::sort(sortedPoly.begin(), sortedPoly.end(), distanceComparator);

    // Take the first 'k' elements from the sorted vector
    std::vector<cv::Point2f> nearestPoints;
    for (int i = 0; i < k && i < sortedPoly.size(); i++) {
        nearestPoints.push_back(sortedPoly[i][0]);
    }

    return nearestPoints;
}

std::vector<std::pair<cv::Point2f, std::vector<cv::Point2f>>> BaseDetection::crossDetection(cv::Mat image) {
    std::vector<std::vector<cv::Point>> contours;
    contours = getContours(image);

    std::vector<std::pair<cv::Point2f, std::vector<cv::Point2f>>> crossCandidates;

    for (const std::vector<cv::Point>& cnt : contours) {
        double perimeter = cv::arcLength(cnt, true);
        std::vector<std::vector<cv::Point2f>> approx;
        cv::approxPolyDP(cnt, approx, 0.01 * perimeter, true);

        std::vector<cv::Point> hull;
        cv::convexHull(cnt, hull, false);
        cv::Mat defects;
        cv::convexityDefects(cnt, hull, defects);

        if (defects.empty()) {
            continue;
        }

        if (8 <= approx.size() && approx.size() <= 14 && !cv::isContourConvex(cnt)) {
            cv::Point2f center = getContourCenter(cnt);
            std::vector<cv::Point2f> fourNearest = getKNearest(approx, center);

            crossCandidates.push_back(std::make_pair(center, fourNearest));
        }
    }

    return crossCandidates;
}

std::vector<std::pair<cv::Point2f, std::vector<cv::Point2f>>> BaseDetection::landingPadDetection(cv::Mat image) {
    std::vector<cv::Point2f> ellipseCandidates = ellipseDetection(image);
    std::vector<std::pair<cv::Point2f, std::vector<cv::Point2f>>> crossCandidates = crossDetection(image);

    std::vector<std::pair<cv::Point2f, std::vector<cv::Point2f>>> landingPads;

    double minDist = 1e7;

    for (const cv::Point2f& ellipseCenter : ellipseCandidates) {
        for (const auto& crossData : crossCandidates) {
            const cv::Point2f& crossCenter = crossData.first;
            const std::vector<cv::Point2f>& points_ = crossData.second;

            double dist = std::pow(ellipseCenter.x - crossCenter.x, 2) + std::pow(ellipseCenter.y - crossCenter.y, 2);

            // Add additional criteria for filtering, e.g., minimum distance, size, etc.
            if (dist < minDist /* Add your filtering criteria here */) {
                landingPads.push_back(std::make_pair(crossCenter, points_));
            }
        }
    }

    return landingPads;
}








