#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <Eigen/Eigen>

class FindingBasesState : public fsm::State {
public:
    FindingBasesState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        Drone* drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("Entering Finding Bases state.");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        

        Drone* drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return "SEG FAULT";

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
};