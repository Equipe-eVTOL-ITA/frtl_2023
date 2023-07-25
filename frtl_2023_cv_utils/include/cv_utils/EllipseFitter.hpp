/*
    ATENÇÃO

    CÓDIGO QUEBRADO!!!!! NÃO USAR EM APLICAÇÃO!!!

    erro: os coeficientes encontrados tem uma ordem de 10^5
    e deveriam ser da ordem da unidade
    suspeita: instabilidade numérica
*/

#ifndef ELLIPSE_FITTER_HPP
#define ELLIPSE_FITTER_HPP
#include "common.hpp"
#include "EllipseFitResult.hpp"
#include <cmath> //verificação de nan
#include <Eigen/Eigen>

struct TreatedPositionData
{
    const Eigen::ArrayXd xs;
    const Eigen::ArrayXd ys;
    const double maxAbsX, maxAbsY;
    const Eigen::Vector2d avgPos;
    TreatedPositionData(Eigen::ArrayXd xs, Eigen::ArrayXd ys,
                            double maxAbsX, double maxAbsY, 
                            Eigen::Vector2d avgPos);
};

class EllipseFitter
{
private:
    double residueThreshold;
public:
    EllipseFitter(double residueThreshold);

    const std::vector<EllipseFitResult> operator()(ContourList contours);

    static cv::Mat displayableResult(const cv::Mat rgbImg, std::vector<EllipseFitResult> fitResults);

    void createTrackbars();

};

#endif