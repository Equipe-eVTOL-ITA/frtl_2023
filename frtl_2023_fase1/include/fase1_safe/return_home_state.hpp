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

        home_pos_ = blackboard.get<Eigen::Vector3d>("Home Position");
        initial_w_ = pos[3];
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        drone->setLocalPositionSync(home_pos[0], home_pos[1], home_pos[2], initial_w_)
    }
    std::string act(fsm::Blackboard &blackboard) override {
    
        Eigen::Vector3d pos  = drone->getLocalPosition(),
                        goal = Eigen::Vector3d({this->initial_x_, this->initial_y_, *h});
        
  

        drone->setLocalPositionSync(home_pos[0], home_pos[1], home_pos[2], initial_w_)

        return "RETURNED HOME";


                if ((pos-goal).norm() < 0.10){
            if (*bases[0].visited && *bases[1].visited)
                return "FINISHED KNOWN BASES";
            return "VISIT NEXT BASE";
        }

        drone->setLocalPosition(this->initial_x, this->initial_y, *h, initial_w_);
        
        return "";
        
    }
private:
    float initial_w_;
    Eigen::Vector3d home_pos_;
};