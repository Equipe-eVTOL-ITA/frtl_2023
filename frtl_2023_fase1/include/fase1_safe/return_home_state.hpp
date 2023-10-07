#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <Eigen/Eigen>

class ReturnHomeState : public fsm::State {
public:
    ReturnHomeState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = blackboard.get<Drone>("drone");
        if (drone_ == nullptr) return;
        drone_->log("Entering Going to Home state.");

        home_pos_ = *blackboard.get<Eigen::Vector3d>("Home Position");
        Eigen::Vector3d pos = drone_->getLocalPosition();
        Eigen::Vector3d orient = drone_->getOrientation();
        initial_w_ = orient[2];
        initial_h_ = pos[2];
    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        Eigen::Vector3d pos = drone_->getLocalPosition();
        drone_->setLocalPositionSync(pos[0], pos[1], home_pos_[2], initial_w_);
        
        drone_->land();
        drone_->disarmSync();
    }
    std::string act(fsm::Blackboard &blackboard) override {
    
        (void)blackboard;
        Eigen::Vector3d pos = drone_->getLocalPosition();
        
        
        if ((pos-home_pos_).norm() < 0.10)
            return "RETURNED HOME";

        drone_->setLocalPosition(home_pos_[0], home_pos_[1], initial_h_, initial_w_);
        
        return "";
    }
private:
    float initial_w_, initial_h_;
    Eigen::Vector3d home_pos_;
    Drone* drone_;
};