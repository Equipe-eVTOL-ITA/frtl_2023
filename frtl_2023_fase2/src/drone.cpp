#include <rclcpp/rclcpp.hpp>
#include "fase2/initial_takeoff_state.hpp"
#include "fase2/return_home_state.hpp"
#include "fase2/read_qrcode_state.hpp"

#include <memory>
#include <iostream>

class Fase2FSM : public fsm::FSM {
public:
    Fase2FSM() : fsm::FSM({"ERROR", "FINISHED"}) {

        this->blackboard_set<Drone>("drone", new Drone());
        this->blackboard_set<cv::Mat>("")

        this->blackboard_set<int>("NumQRCodeLidos", 0);

        Drone* drone = blackboard_get<Drone>("drone");
        drone->create_image_publisher("/qrcode");

        this->add_state("INITIAL TAKEOFF", std::make_unique<InitialTakeoffState>());
        this->add_state("READ QRCODE", std::make_unique<ReadQRCodeState>());
        this->add_state("TRAVEL", std::make_unique<InitialTakeoffState>());
        this->add_state("RETURN HOME", std::make_unique<ReturnHomeState>());
        this->add_state("FIND QRCODE", std::make_unique<FindQRCodeState>());

        this->add_transitions("INITIAL TAKEOFF", {{"INITIAL TAKEOFF COMPLETED", "FIND QRCODE"},{"SEG FAULT", "ERROR"}});
        this->add_transitions("FIND QRCODE", {{"FOUND QRCODE", "TRAVEL"}, {"SEG FAULT", "ERROR"}});
        this->add_transitions("TRAVEL", {{"POSITION REACHED", "READ QRCODE"}, {"SEG FAULT", "ERROR"}}); 

        this->add_transitions("READ QRCODE", {{"QRCODE SUCCESSFULLY READ", "FIND QRCODE"}, {"SEG FAULT", "ERROR"}});

        this->add_transitions("READ QRCODE", {{"ALL QRCODE READ", "RETURN HOME"}, {"SEG FAULT", "ERROR"}})
        
    }
}

class NodeFSM : public rclcpp::Node {
public:
    NodeFSM() : rclcpp::Node("fase2_node") {}
    Fase2FSM my_fsm;
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