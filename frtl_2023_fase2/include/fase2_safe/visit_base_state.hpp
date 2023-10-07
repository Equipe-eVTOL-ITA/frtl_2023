#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "fase2_safe/base.hpp"
#include <Eigen/Eigen>

class VisitBaseState : public fsm::State {
public:
    VisitBaseState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        Drone* drone_ = blackboard.get<Drone>("drone");
        if (drone_ == nullptr) return;
        drone_->log("Entering Visit Base state.");
        
        bases_ = blackboard.get<std::vector<Base>>("Bases");
        
        index_base_to_visit_ = 0;
        while (index_base_to_visit_< (*bases_).size()){
            if (!(*bases_)[index_base_to_visit_].visited){
                x_target_ = (((*bases_)[index_base_to_visit_]).coordinates).x;
                y_target_ = (((*bases_)[index_base_to_visit_]).coordinates).y;
                break;
            }
            index_base_to_visit_++;
        }
        if (index_base_to_visit_ >= (*bases_).size()){
            drone_->log("Ended visiting all QR Codes.");
            return;
        }


        Eigen::Vector3d pos = drone_->getLocalPosition();
        Eigen::Vector3d orient = drone_->getOrientation();
        this->initial_h_ = pos[2];
        this->initial_w_ = orient[2];
    }

    std::string act(fsm::Blackboard &blackboard) override {
        
        (void)blackboard;

        Eigen::Vector3d pos  = drone_->getLocalPosition(),
                        goal = Eigen::Vector3d({x_target_, y_target_, initial_h_});

        
        if ((pos-goal).norm() < 0.10){
            for(int i = 0; i < 5; i++){
                if ((*bases_)[0].visited && (*bases_)[1].visited)
                    return "FINISHED BASES";
            }
            return "VISIT NEXT BASE";
        }

        drone_->setLocalPosition(x_target_, y_target_, initial_h_, initial_w_);
        
        return "";
    }
    
private:
    int index_base_to_visit_;
    float initial_h_, initial_w_;
    float x_target_, y_target_;
    std::vector<Base> *bases_;
    Drone* drone_;

};