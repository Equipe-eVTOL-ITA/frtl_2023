#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <Eigen/Eigen>

class LandState : public fsm::State {
public:
    LandState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        Drone* drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("Entering landing state.");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        

        Drone* drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return "SEG FAULT";

        Eigen::Vector3d pos = drone->getLocalPosition();
        float diff = pos[2] - ground_height;
        if(diff < 0) diff = -diff;

        drone->land();
        if (diff < 0.1)
            return "LAND COMPLETED";

        return "";
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        Drone* drone = blackboard.get<Drone>("drone");
        drone->log("landed");
        drone->disarmSync();
    }

private:
    float ground_height = 0.1;
};