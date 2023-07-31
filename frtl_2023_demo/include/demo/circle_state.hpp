#include "fsm/fsm.hpp"

#include "drone/Drone.hpp"
#include <Eigen/Eigen>
#include <cmath>

class CircleState : public fsm::State {
public:
    CircleState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {

        this->drone_ = blackboard.get<Drone>("drone");
        this->drone_->log("Entering circle state.");

        this->r_ = *blackboard.get<float>("radius");
        this->w_ = *blackboard.get<float>("angular");
        this->h_ = *blackboard.get<float>("height");
        this->timeout_ = 3 * 2 * acos(-1) / this->w_;
        this->start_time_ = this->drone_->getTime();
        
        Eigen::Vector3d position = this->drone_->getLocalPosition();
        this->x_center_ = position[0] - this->r_;
        this->y_center_ = position[1];

    }

    void on_exit(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        Eigen::Vector3d position = this->drone_->getLocalPosition();
        int i = 1e4; 
        while (i--)
            this->drone_->setLocalPosition(position[0], position[1], h_, 0.0);


    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        if (drone_ == nullptr) return "SEG FAULT";

        double dt = this->drone_->getTime() - this->start_time_;
        if (dt > this->timeout_)
            return "CIRCLE COMPLETED";

        float x = r_ * cos(w_ * dt) + x_center_;
        float y = r_ * sin(w_ * dt) + y_center_;

        this->drone_->setLocalPosition(x, y, h_, 0.0);
        
        return "";
    }

private:
    double start_time_;
    float timeout_;
    float r_; // radius
    float w_; // angular velocity 
    float h_; // height
    float x_center_;
    float y_center_;
    Drone *drone_;
};