#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <cv_utils/ImgToWorld.cpp>
#include <cv_utils/BaseDetector.cpp>

class FindingBasesState : public fsm::State {
public:
    FindingBasesState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        Drone* drone = blackboard.get<Drone>("drone");

        if (drone == nullptr) return;
        drone->log("Entering Finding Bases state.");

        this->start_time_ = this->drone_->getTime();

        this->cv_img_mode = *blackboard.get<std::string>("Horizontal or Vertical CV");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        

        Drone* drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return "SEG FAULT";

        double duration = this->drone_->getTime() - this->start_time_;

        cv_bridge::CvImagePtr ver_ptr = drone->getVerticalImage();
        cv_bridge::CvImagePtr hor_ptr = drone->getHorizontalImage();
        

        




        /*
        if (basesFound > 0 && timedOut == true)
            return "FOUND BASES";
        else if(basesFound = 0 && timedOut == true)
            return "TIMED OUT";

        // Entrar em modo de procura das bases

        return "";
        */
        return "FOUND BASES";
    }

    
private:
    double start_time_;
    std::string cv_img_mode;
};