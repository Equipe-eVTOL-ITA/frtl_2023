/*
    ATENÇÃO

    CÓDIGO QUEBRADO!!!!! NÃO USAR EM APLICAÇÃO!!!

    erro: os coeficientes encontrados tem uma ordem de 10^5
    e deveriam ser da ordem da unidade
    suspeita: instabilidade numérica
*/

#include <cv_utils/EllipseFitter.hpp>

TreatedPositionData::TreatedPositionData(Eigen::ArrayXd xs, Eigen::ArrayXd ys,
                            double maxAbsX, double maxAbsY, 
                            Eigen::Vector2d avgPos)
    : xs{xs}, ys{ys}, maxAbsX{maxAbsX}, maxAbsY{maxAbsY}, avgPos{avgPos}
{}

EllipseFitter::EllipseFitter(double residueThreshold)
    : residueThreshold{residueThreshold}
{}

//transformar em overload do construtor 
TreatedPositionData treatPositionData(std::vector<cv::Point2i> contour) {
    int nPoints = contour.size();

    double avgX=0, avgY=0;
    for (auto pointIt = contour.begin();
             pointIt < contour.end(); pointIt++) {
        avgX += pointIt->x;
        avgY += pointIt->y;
    }
    avgX /= nPoints;
    avgY /= nPoints;

    Eigen::ArrayXd xs(nPoints, 1);
    Eigen::ArrayXd ys(nPoints, 1);

    double maxAbsX = 0, maxAbsY = 0;
    int i = 0;
    for (auto pointIt = contour.begin(); pointIt < contour.end(); pointIt++) {
        xs(i) = pointIt->x - avgX;
        ys(i) = pointIt->y - avgY;
        if (abs(xs(i)) > maxAbsX) maxAbsX = abs(xs(i));
        if (abs(ys(i)) > maxAbsY) maxAbsY = abs(ys(i));
    }
    xs /= maxAbsX;
    ys /= maxAbsY;
    return TreatedPositionData(xs, ys, maxAbsX, maxAbsY, Eigen::Vector2d(avgX, avgY));
}
#include <iostream>
const std::vector<EllipseFitResult> EllipseFitter::operator()(ContourList contours) {
    std::vector<EllipseFitResult> results;

    for (auto it = contours.begin(); it < contours.end(); it++) {
        TreatedPositionData positionData = treatPositionData(*it);
        //contorno é muito pequeno
        if (positionData.maxAbsX < 1e-2 || positionData.maxAbsY < 1e-2)
            continue;

        //https://eigen.tuxfamily.org/dox/group__LeastSquares.html
        Eigen::MatrixXd A(it->size(), 3);
        A << Eigen::VectorXd(positionData.xs * positionData.xs),
             Eigen::VectorXd(positionData.ys * positionData.xs),
             Eigen::VectorXd(positionData.ys * positionData.ys);

        Eigen::VectorXd b(it->size());
        b.fill(1);

        Eigen::Vector3d coefs = (A.transpose() * A).ldlt().solve(A.transpose() * b);

        //verificar se há coef = nan ou se o limite de resíduo foi satisfeito
        if (std::isnan(coefs(0)) || std::isnan(coefs(1)) || std::isnan(coefs(2)) ||
                                    (b - A * coefs).squaredNorm() >= residueThreshold)
            continue;
        
        results.push_back(EllipseFitResult(coefs, positionData.avgPos, {positionData.maxAbsX, positionData.maxAbsY}));
    }
    return results;
}

double normToReal(double norm, double m, double c) {
    return norm * m + c;
}

double deltaSqrt(double a, double b, double c) {
    return sqrt(b*b - 4 * a * c);
}

double bhaskara(double a, double b, double c, bool positive) {
    double deltaRoot = deltaSqrt(a, b, c);
    return (-b + ((positive) ? 1 : -1) * deltaRoot) / (2 * a);
}

cv::Mat EllipseFitter::displayableResult(const cv::Mat rgbImg, std::vector<EllipseFitResult> fitResults) {
    cv::Mat displayImage = rgbImg.clone();
    for (auto fitResultIt = fitResults.begin(); fitResultIt < fitResults.end(); fitResultIt++) {
        // std::vector<cv::Point2i> ellipsePoints;
        // double a = fitResultIt->coefs(0);
        // double b = fitResultIt->coefs(1);
        // double c = fitResultIt->coefs(2);
        // for (int i = 0; i <= 50; i++) {
        //     double xNormalized  = -1 + 2 * (double) i / 50.0f;

        //     double yNormalizedPositive = bhaskara(c, b*xNormalized, a*xNormalized*xNormalized - 1, true);
        //     double yNormalizedNegative = bhaskara(c, b*xNormalized, a*xNormalized*xNormalized - 1, false);

        //     double xReal         = normToReal(xNormalized, fitResultIt->maxCoords(0), fitResultIt->centroid(0));
        //     double yRealPositive = normToReal(yNormalizedPositive, fitResultIt->maxCoords(1), fitResultIt->centroid(1));
        //     double yRealNegative = normToReal(yNormalizedNegative, fitResultIt->maxCoords(1), fitResultIt->centroid(1));
            
        //     ellipsePoints.push_back({(int)xReal, (int)yRealPositive});
        //     ellipsePoints.insert(ellipsePoints.begin(), {(int)xReal, (int)yRealNegative});
        // }
        // cv::polylines(displayImage, ellipsePoints, true, {0,0,0}, 3);
        cv::circle(displayImage, {(int) fitResultIt->centroid(0), (int) fitResultIt->centroid(1)},
                 10, {0,0,0}, cv::FILLED);
    }
    return displayImage;
}

void EllipseFitter::createTrackbars() {
    std::string windowName = "Ellipse fitter trackbars";
    cv::namedWindow(windowName);
    TRACKBAR(residueThreshold, windowName, EllipseFitter, double, 1000)
}