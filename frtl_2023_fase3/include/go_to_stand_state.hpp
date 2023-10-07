#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <Eigen/Eigen>

class GoToStandState : public fsm::State {
public:
        GoToStandState() : fsm::State() {}

        void on_enter(fsm::Blackboard &blackboard) override {
                
                Drone* drone = blackboard.get<Drone>("drone");
                if (drone == nullptr) return;
                drone->log("Initial taking off.");

                drone->toOffboardSync();
                drone->armSync();
                
                Eigen::Vector3d pos = drone->getLocalPosition();
                this->initial_x = pos[0];
                this->initial_y = pos[1];
        }

        std::string act(fsm::Blackboard &blackboard) override {

                if (drone == nullptr) return "SEG FAULT";

                float* z =  blackboard.get<float>("height");
                if (z == nullptr) return "SEG FAULT";

                Eigen::Vector3d pos  = drone->getLocalPosition(),
                        goal = Eigen::Vector3d({this->initial_x, this->initial_y, *z});

                if ((pos-goal).norm() < 0.10)
                        return "INITIAL TAKEOFF COMPLETED";

                drone->setLocalPosition(this->initial_x, this->initial_y, *z, 0.0);
                Eigen::Vector3d position = this->drone_->getLocalPosition();

        }

        float initial_x, initial_y;
};

// va para (x, y) - medir posicao que camera enxerga estante completa 

// reconhecer estante - atribuir limites laterais e distancia entre prateleiras (seguir cano)

// transicao para "go to first shelf"