#include <cv_utils/PointCloudTracker.hpp>

//inicializa counter com 0
unsigned int Target::counter = 0;

PointCloudTracker::PointCloudTracker(float alpha, unsigned int maxFramesWoDetection, float maxDeltaPosition)
    : alpha{alpha}, maxFramesWoDetection{maxFramesWoDetection}, maxDeltaPosition{maxDeltaPosition}
{}

//simplificar!!!
std::vector<Target> PointCloudTracker::operator()(std::vector<Eigen::Vector2d> newMeasurements) {
    int nMeas = newMeasurements.size();
    int nTarg = trackedTargets.size();
    
    //1 linha/target, 1 coluna/measurement
    //+1 evita erro quando nTarg e nMeas = 0, sem precisar de ifs infinitos
    Eigen::MatrixXd targetToMeasDists(nTarg+1, nMeas+1);
    targetToMeasDists.fill(INFINITY);   //será que funciona?

    //popular vetor de distancias
    int targInd = 0;
    int measInd = 0;
    for (auto targIt = trackedTargets.begin();
            targIt < trackedTargets.end(); targIt++)
    {
        measInd = 0;
        for (auto measIt = newMeasurements.begin();
                measIt < newMeasurements.end(); measIt++)
        {
            targetToMeasDists(targInd, measInd) = 
                            (*measIt - targIt->imgPos).norm();
            measInd++;
        }
        targInd++;
    }

    //atualiza alvos já existentes
    targInd = 0;
    for (auto targIt = trackedTargets.begin(); 
            targIt < trackedTargets.end();) {
        Eigen::Index minDistMeasIndex;
        double minDist = targetToMeasDists.row(targInd).minCoeff(&minDistMeasIndex);

        //atualiza com filtro exponencial
        if (minDist <= maxDeltaPosition) {
            Eigen::Vector2d newPositionMeas = newMeasurements.at(minDistMeasIndex);
            targIt->imgPos = (1 - alpha) * targIt->imgPos + alpha * newPositionMeas;
        }
        else {
            targIt->framesWoDetection++;
        }

        //remove elemento se necessário
        if (targIt->framesWoDetection >= maxFramesWoDetection)
            targIt = trackedTargets.erase(targIt);
        else
            targIt++;
    }

    //adiciona novos alvos
    measInd = 0;
    for (auto measIt = newMeasurements.begin(); measIt < newMeasurements.end(); measIt++) {
        double minDist = targetToMeasDists.col(measInd).minCoeff();

        //medida é um novo alvo
        if (minDist > maxDeltaPosition) {
            trackedTargets.push_back(Target(*measIt));
        }
    }

    return trackedTargets;
}

cv::Mat PointCloudTracker::displayableResult(const cv::Mat rgbImg, std::vector<Target> targets) {
    cv::Mat displayImage = rgbImg.clone();
    for (auto targIt = targets.begin(); targIt < targets.end(); targIt++) {
        cv::Point target(targIt->getImgPos()(0), targIt->getImgPos()(1));
        cv::circle(displayImage, target,
                 5, {255, 255, 255}, cv::FILLED);
        cv::putText(displayImage, std::to_string(targIt->getId()), target, cv::FONT_HERSHEY_SIMPLEX,
                1, {255, 255, 255}, 3);
    }
    return displayImage;
}