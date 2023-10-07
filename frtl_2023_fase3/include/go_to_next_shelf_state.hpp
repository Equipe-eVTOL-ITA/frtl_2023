#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <Eigen/Eigen>

class GoToNextShelfState : public fsm::State {
public:
        GoToNextShelfState() : fsm::State() {}

        void on_enter() override {
                Drone* drone = blackboard.get<Drone>("drone");
                if (drone == nullptr) return;

                Eigen::Vector3d initial_position = this->drone_->getLocalPosition();
                initial_z = initial_position[2];

                shelfHeight = 0.5;

                int *direction = blackboard.get<int>("DIRECTION");
                *direction = (*direction+1)%2;
        }

        std::string act(fsm::Blackboard &blackboard) override {
                if (drone == nullptr) return "SEG FAULT";

                Eigen::Vector3d position = this->drone_->getLocalPosition();

                if(abs(position[2])<1.5*shelfHeight) return "RETURN HOME";

                drone->setLocalPosition(position[0], position[1], initial_z+shelfHeight, 0.0);

                if(position[2]-initial_z-shelfHeight < 0.1) return "READ SHELF";

                return "";
        }

        int shelfHeight;
        int initial_z;
};