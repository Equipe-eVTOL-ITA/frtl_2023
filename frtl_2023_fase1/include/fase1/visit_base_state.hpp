#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <Eigen/Eigen>

class VisitBaseState : public fsm::State {
public:
    VisitBaseState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        Drone* drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("Entering Visit Base state.");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        

        Drone* drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return "SEG FAULT";

        // Enviar o drone para a Base

        /*
        if (chegou na base)
            return "ARRIVED AT BASE";
        */
       return "ARRIVED AT BASE";
    }
};