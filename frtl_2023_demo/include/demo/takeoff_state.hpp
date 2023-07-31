#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <Eigen/Eigen>

class TakeoffState : public fsm::State {
public:
    TakeoffState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {

        Drone* drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("Taking off.");

        Eigen::Vector3d pos = drone->getLocalPosition();
        this->x_ = pos[0];
        this->y_ = pos[1];

        drone->toOffboardSync();
        drone->armSync();
    }

    std::string act(fsm::Blackboard &blackboard) override {
        

        Drone* drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return "SEG FAULT";

        float* z =  blackboard.get<float>("height");
        if (z == nullptr) return "SEG FAULT";

        Eigen::Vector3d pos  = drone->getLocalPosition(),
                        goal = Eigen::Vector3d({x_,y_,*z});

        if ((pos-goal).norm() < 0.10)
            return "TAKEOFF COMPLETED";

        drone->setLocalPosition(this->x_, this->y_, *z, 0.0);
        
        return "";
    }

    float x_, y_;
};