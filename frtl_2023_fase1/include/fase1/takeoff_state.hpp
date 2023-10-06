#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <Eigen/Eigen>

#include <opencv2/highgui.hpp>

class TakeoffState : public fsm::State {
public:
    TakeoffState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {

        Drone* drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("Taking off.");

        drone->toOffboardSync();
        drone->armSync();
        
        Eigen::Vector3d pos = drone->getLocalPosition();
        this->initial_x = pos[0];
        this->initial_y = pos[1];
    }

    std::string act(fsm::Blackboard &blackboard) override {
        

        Drone* drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return "SEG FAULT";

        float* z =  blackboard.get<float>("height");
        if (z == nullptr) return "SEG FAULT";

        Eigen::Vector3d pos  = drone->getLocalPosition(),
                        goal = Eigen::Vector3d({this->initial_x, this->initial_y, *z});

        
        /*
        if ((pos-goal).norm() < 0.10 && visited_bases == known_bases)
            return "FINISHED KNOWN BASES";
        else if ((pos-goal).norm() < 0.10 && visited_bases < known_bases)
            return "NEXT BASE";
        */
        if ((pos-goal).norm() < 0.10)
            return "FINISHED KNOWN BASES";

        drone->setLocalPosition(this->initial_x, this->initial_y, *z, 0.0);

        cv_bridge::CvImagePtr cv_ptr = drone->getVerticalImage();
        cv::Mat edged_image;
        cv::Canny(cv_ptr->image, edged_image, 100, 200);
        drone->publish_image("/transformed_vertical_image", edged_image);
        // drone-> publish
        
        return "";
    }

    float initial_x, initial_y;
};