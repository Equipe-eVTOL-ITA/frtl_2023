#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <Eigen/Eigen>

class ReturnHomeState : public fsm::State {
public:
    ReturnHomeState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        Drone* drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("Entering Going to Home state.");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        

        Drone* drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return "SEG FAULT";

        // Enviar o drone para Home

        return "RETURNED HOME";
        
    }
};