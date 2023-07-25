#ifndef POINT_CLOUD_TRACKER_HPP
#define POINT_CLOUD_TRACKER_HPP
#include "common.hpp"
#include <algorithm>

struct Target
{
private:
    Eigen::Vector2d imgPos;
    unsigned int framesWoDetection;
    static unsigned int counter;
    unsigned int id; //queria que fosse public const :(
public:
    Eigen::Vector2d getImgPos() {return imgPos;}
    unsigned int getId() {return id;}
    Target(Eigen::Vector2d imgPos)
    : imgPos{imgPos}, framesWoDetection{0}, id{counter++}
    {}
    friend class PointCloudTracker;
};


class PointCloudTracker
{
private:
    float alpha;
    unsigned int maxFramesWoDetection;
    float maxDeltaPosition;
    std::vector<Target> trackedTargets;
public:
    PointCloudTracker(float alpha, unsigned int maxFramesWoDetection, float maxDeltaPosition);

    std::vector<Target> operator()(std::vector<Eigen::Vector2d> newMeasurements);

    static cv::Mat displayableResult(const cv::Mat rgbImg, std::vector<Target> targets);

    void createTrackbars();

};

#endif