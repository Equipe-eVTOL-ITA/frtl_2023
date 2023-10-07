#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "fase1_safe/base.hpp"
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
        while (index_base_to_visit_ < (*bases_).size()){
            if (!(*bases_)[index_base_to_visit_].visited){
                blackboard.set<float>("Height of Base", (((*bases_)[index_base_to_visit_]).coordinates).z);
                break;
            }
        }
        if (index_base_to_visit_ >= (*bases_).size()){
            drone_->log("Something is wrong with .visited state of bases.");
            return;
        }

        Eigen::Vector3d pos = drone_->getLocalPosition();
        this->initial_h_ = pos[2];
        this->initial_w_ = pos[3];
    }

    std::string act(fsm::Blackboard &blackboard) override {

        (void)blackboard;
        Eigen::Vector3d pos  = drone_->getLocalPosition(),
                        goal = Eigen::Vector3d({(((*bases_)[index_base_to_visit_]).coordinates).x, 
                                                (((*bases_)[index_base_to_visit_]).coordinates).y,
                                                initial_h_});

        if ((pos-goal).norm() < 0.10){
            if ((*bases_)[0].visited && (*bases_)[1].visited)
                return "FINISHED BASES";
            return "VISIT NEXT BASE";
        }

        drone_->setLocalPosition((*bases_)[index_base_to_visit_].coordinates.x,
                                (*bases_)[index_base_to_visit_].coordinates.y,
                                initial_h_,
                                initial_w_);
        
        return "";
    }
private:
    int index_base_to_visit_;
    float initial_h_, initial_w_;
    std::vector<Base> *bases_;
    Drone* drone_;

};