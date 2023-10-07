#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "fase1_safe/base.hpp"
#include <Eigen/Eigen>

#include <opencv2/highgui.hpp>

class TakeoffState : public fsm::State {
public:
    TakeoffState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {

        drone_ = blackboard.get<Drone>("drone");
        if (drone_ == nullptr) return;
        drone_->log("Taking off.");

        bases_ = blackboard.get<std::vector<Base>>("Bases");
        h_ = blackboard.get<float>("height");

        drone_->toOffboardSync();
        drone_->armSync();
        
        Eigen::Vector3d pos = drone_->getLocalPosition();
        Eigen::Vector3d orient = drone_->getOrientation();
        this->initial_x_ = pos[0];
        this->initial_y_ = pos[1];
        this->initial_w_ = orient[2];
    }

    std::string act(fsm::Blackboard &blackboard) override {

        (void)blackboard;
        Eigen::Vector3d pos  = drone_->getLocalPosition(),
                        goal = Eigen::Vector3d({this->initial_x_, this->initial_y_, *h_});

        if ((pos-goal).norm() < 0.10){
            if ((*bases_)[0].visited && (*bases_)[1].visited)
                return "FINISHED BASES";
            return "VISIT NEXT BASE";
        }

        drone_->setLocalPosition(this->initial_x_, this->initial_y_, *h_, initial_w_);
        
        return "";
    }
    
private:
    float initial_x_, initial_y_, initial_w_;
    float* h_;
    Drone* drone_;
    std::vector<Base>* bases_;
};