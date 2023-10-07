#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <Eigen/Eigen>
#include <iostream>

class FindQRCodeState : public fsm::State{
public:
    FindQRCodeState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
         Drone* drone = blackboard.get<Drone>("drone");

        if (drone == nullptr) return;
        drone->log("Entering Finding Bases state.");
    }

    std::string act(fsm::Blackboard &blackboard) override {

        Drone* drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return "SEG FAULT";

        cv_bridge::CvImagePtr ver_ptr = drone->getVerticalImage();
        cv_bridge::CvImagePtr hor_ptr = drone->getHorizontalImage();

        return "FOUND QRCODE";
    }
}