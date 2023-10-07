#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <Eigen/Eigen>
#include <cv_utils/Qrcode.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

class ReadQRCodeState : public fsm::State{
public:
    ReadQRCodeState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {

        Drone* drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return;
        drone->log("Reading qrcode.");
   }

    std::string act(fsm::Blackboard &blackboard) override {

        Drone* drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return "SEG FAULT";

        auto cv_ptr = drone->getVerticalImage();
        std::string qrCodeData = qrCode(cv_ptr->image);

        int NumQRCodeLidos = blackboard.get<int>("NumQRCodeLidos");

        if (NumQRCodeLidos != 5){
            if (!qrCodeData.empty()){
            NumQRCodeLidos += 1;
            drone->log (qrCodeData);
            return "QRCODE SUCCESSFULLY READ";
            } else {
            return "";
            }
        } else {
            return "ALL QRCODE READ";
        }
    }
}