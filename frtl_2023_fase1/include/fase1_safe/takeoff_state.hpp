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

        bases = blackboard.get<std::vector<Bases>>("Bases");
        h_ = blackboard.get<float>("height");

        drone->toOffboardSync();
        drone->armSync();
        
        Eigen::Vector3d pos = drone->getLocalPosition();
        this->initial_x_ = pos[0];
        this->initial_y_ = pos[1];
        this->initial_w_ = pos[3];
    }

    std::string act(fsm::Blackboard &blackboard) override {
        

        Drone* drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return "SEG FAULT";

        Eigen::Vector3d pos  = drone->getLocalPosition(),
                        goal = Eigen::Vector3d({this->initial_x_, this->initial_y_, *h});

        if ((pos-goal).norm() < 0.10){
            if (*bases[0].visited && *bases[1].visited)
                return "FINISHED KNOWN BASES";
            return "VISIT NEXT BASE";
        }

        drone->setLocalPosition(this->initial_x, this->initial_y, *h, initial_w_);
        
        return "";
    }
private:
    float initial_x_, initial_y_, initial_w_;
    float* h_;
};