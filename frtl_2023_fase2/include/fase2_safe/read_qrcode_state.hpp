#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "fase2_safe/base.hpp"
#include <Eigen/Eigen>
#include "cv_utils/qrcode_reader.hpp"

class ReadingState : public fsm::State {
public:
    ReadingState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {

        drone_ = blackboard.get<Drone>("drone");
        if (drone_ == nullptr) return;
        drone_->log("Taking off.");

        bases_ = blackboard.get<std::vector<Base>>("Bases");
        
        Eigen::Vector3d pos = drone_->getLocalPosition();
        Eigen::Vector3d orient = drone_->getOrientation();
        this->initial_x_ = pos[0];
        this->initial_y_ = pos[1];
        this->initial_w_ = orient[2];
        this->target_h_ = *blackboard.get<float>("Height to Read QR Code");
    }

    std::string act(fsm::Blackboard &blackboard) override {

        (void)blackboard;
        Eigen::Vector3d pos  = drone_->getLocalPosition(),
                        goal = Eigen::Vector3d({this->initial_x_, this->initial_y_, target_h_});

        if ((pos-goal).norm() < 0.40){
            bool got_qrcode = false;
            reader.detectQRCode(drone_->getVerticalImage()->image, got_qrcode);
            got_qrcode = true;
            if (got_qrcode)
                return "DONE READING";
        }

        drone_->setLocalPosition(this->initial_x_, this->initial_y_, target_h_, this->initial_w_);
        
        return "";
    }

private:
    float initial_x_, initial_y_, initial_w_, target_h_;
    Drone* drone_;
    std::vector<Base>* bases_;
    QRCodeReader reader;
};