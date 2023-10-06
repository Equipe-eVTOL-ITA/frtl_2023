#include "fase1/landing_state.hpp"
#include "fase1/initial_takeoff_state.hpp"
#include "fase1/takeoff_state.hpp"
#include "fase1/visit_base_state.hpp"
#include "fase1/return_home_state.hpp"
#include "fase1/finding_bases_state.hpp"
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <iostream>

class Fase1FSM : public fsm::FSM {
public:
    Fase1FSM() : fsm::FSM({"ERROR", "FINISHED"}) {

        this->blackboard_set<Drone>("drone", new Drone());
        this->blackboard_set<cv::Mat>("")

        Drone* drone = blackboard_get<Drone>("drone");
        drone->create_image_publisher("/transformed_vertical_image");
        drone->create_image_publisher("/transformed_vertical_image");

        this->blackboard_set<float>("Horizontal or Vertical CV", "vertical");

        this->add_state("INITIAL TAKEOFF", std::make_unique<InitialTakeoffState>());
        this->add_state("FINDING BASES", std::make_unique<FindingBasesState>());
        this->add_state("RETURN HOME", std::make_unique<ReturnHomeState>());
        this->add_state("VISIT BASE", std::make_unique<VisitBaseState>());
        this->add_state("TAKEOFF", std::make_unique<TakeoffState>());
        this->add_state("LANDING", std::make_unique<LandingState>());

        this->add_transitions("INITIAL TAKEOFF", {{"INITIAL TAKEOFF COMPLETED", "FINDING BASES"},{"SEG FAULT", "ERROR"}});



        
        this->add_transitions("FINDING BASES", {{"FOUND BASES", "VISIT BASE"},{"SEG FAULT", "ERROR"}});
        this->add_transitions("VISIT BASE", {{"ARRIVED AT BASE", "LANDING"},{"SEG FAULT", "ERROR"}});
        this->add_transitions("LANDING", {{"LANDED", "TAKEOFF"},{"SEG FAULT", "ERROR"}});
        //Transicao da Takeoff
        this->add_transitions("TAKEOFF", {{"NEXT BASE", "VISIT BASE"},{"SEG FAULT", "ERROR"}});
        this->add_transitions("TAKEOFF", {{"FINISHED KNOWN BASES", "FINDING BASES"},{"SEG FAULT", "ERROR"}});
        // -------------------
        this->add_transitions("RETURN HOME", {{"RETURNED HOME", "FINISHED"},{"SEG FAULT", "ERROR"}});

        //TRANSICOES DE TIME OUT
        this->add_transitions("FINDING BASES", {{"TIME OUT", "RETURN HOME"},{"SEG FAULT", "ERROR"}});
        this->add_transitions("VISIT BASE", {{"TIME OUT", "RETURN HOME"},{"SEG FAULT", "ERROR"}});

    }
};

class NodeFSM : public rclcpp::Node {
public:
    NodeFSM() : rclcpp::Node("fase1_node") {}
    Fase1FSM my_fsm;
};

int main(int argc, const char * argv[]){
    rclcpp::init(argc,argv);

    auto my_node = std::make_shared<NodeFSM>();
    while (rclcpp::ok() && !my_node->my_fsm.is_finished()) {
        my_node->my_fsm.execute();
        rclcpp::spin_some(my_node);
    }

    std::cout << my_node->my_fsm.get_fsm_outcome() << std::endl;
    rclcpp::shutdown();
    
    return 0;
}





/*
#include "fase1/drone.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <iostream>

#include <iostream>
#include <string>



class node : rclcpp::Node() 
public:
    execute() {

        auto cv_br = drone.getVerticalCamera()

        extract_edges(out, cv_br.image)

    }


class NodeFSM : public rclcpp::Node {
public:
    Drone drone;
    NodeFSM() : rclcpp::Node("fase1_node") {
        drone.create_image_publisher("/my_image");

    }
    void execute(){
        auto extrac_img = drone.getVerticalImage();
        drone.publish_image("/my_image", extrac_img);
        //cv::imwrite("image.jpg", extrac_img->image);
        
        //cv::imwrite("yellowboxfilter.jpg", yellowFilter(extrac_img->image));
        //yellowFilter.apply(extrac_img->image)
    }
};

int main(int argc, const char * argv[]){
    std::cout << "ta funcionando\n";
    rclcpp::init(argc,argv);
    
    rclcpp::Rate loop_rate(60);
    auto my_node = std::make_shared<NodeFSM>();
    while (rclcpp::ok()) {
        my_node->execute();
        rclcpp::spin_some(my_node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    
    return 0;
    //cv::Mat rgb = cv::imread("../../fase1/marco_pouso.jpg");
    //cv::imwrite("../../fase1/yellowboxfilter.jpg", yellowFilter(rgb));
}

*/