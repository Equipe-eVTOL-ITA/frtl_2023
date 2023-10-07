#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <cv_utils/ImgToWorld.cpp>
#include <cv_utils/BaseDetector.cpp>

class FindingBasesState : public fsm::State {
public:
    FindingBasesState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        Drone* drone = blackboard.get<Drone>("drone");
        time_limit_ = blackboard.get<double>("time limit");

        camera_params = blackboard.get<cv::Mat>("Camera Parameters");

        std::vector<Point3f>> *bases = blackboard.get<std::vector<Point3f>>("Bases");

        if (drone == nullptr) return;
        drone->log("Entering Finding Bases state.");

        this->start_time_ = this->drone_->getTime();

        this->cv_img_mode_ = *blackboard.get<std::string>("Horizontal or Vertical CV");

        did_detect_base_ = false;

        
    }

    std::string act(fsm::Blackboard &blackboard) override {
        

        Drone* drone = blackboard.get<Drone>("drone");
        if (drone == nullptr) return "SEG FAULT";

        double duration = this->drone_->getTime() - this->start_time_;

        cv_bridge::CvImagePtr ver_ptr = drone->getVerticalImage();
        cv_bridge::CvImagePtr hor_ptr = drone->getHorizontalImage();


        std::vector<std::pair<cv::Point2f, std::vector<cv::Point2f>>> ver_img_coordinates = baseDetection(ver_prt->image);
        std::vector<std::pair<cv::Point2f, std::vector<cv::Point2f>>> ver_img_coordinates = baseDetection(hor_prt->image);



        for(ver_img_coord : ver_img_coordinates){
            ver_coordinates_ = ImageToWorld(ver_img_coord[i].second, .......);

            int i = 1;
            while (i < 6 && (ver_coordinates_ - *bases[i].coordinates).norm() < 0.10)
                i++; 

            if(i < 6){
                
                
                did_detect_base_ = true;
                i = 1;
                while (*bases[i].found == true)
                    i++;
                *bases[i] = {ver_coordinates_, true, false};
            }
        }

        if (this->did_detect_base_ == true && duration > time_limit_){
            drone->log("Found " + std::string(num_bases) + " bases.");
            return "FOUND BASES";
        }
        else if(this->did_detect_base_ == false && duration > time_limit_){
            drone->log("Timed out searching for bases.");
            return "TIMED OUT";
        }            

        return "";
    }

    
private:
    double start_time_;
    std::string cv_img_mode_;
    bool did_detect_base_;
    double time_limit_;
    std::vector<cv::Point3f>& object_points_;
    std::vector<cv::Point2f>& image_points_,
    Eigen::Vector3d ver_coordinates_;

};