#include "demo/takeoff_state.hpp"
#include "demo/land_state.hpp"
#include "demo/circle_state.hpp"
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <iostream>

class CircleFSM : public fsm::FSM {
public:
    CircleFSM() : fsm::FSM({"ERROR", "FINISHED"}) {

        this->blackboard_set<float>("height", -5.0);
        this->blackboard_set<float>("radius", 5.0);
        this->blackboard_set<float>("angular", 1.0);
        this->blackboard_set<Drone>("drone", new Drone());

        Drone* drone = blackboard_get<Drone>("drone");
        drone->create_image_publisher("/transformed_vertical_image");


        this->add_state("TAKEOFF", std::make_unique<TakeoffState>());
        this->add_transitions("TAKEOFF", {{"TAKEOFF COMPLETED", "CIRCLE"},{"SEG FAULT", "ERROR"}});
            
        this->add_state("CIRCLE", std::make_unique<CircleState>());
        this->add_transitions("CIRCLE", {{"CIRCLE COMPLETED", "LAND"},{"SEG FAULT", "ERROR"}});

        this->add_state("LAND", std::make_unique<LandState>());
        this->add_transitions("LAND", {{"LAND COMPLETED", "FINISHED"},{"SEG FAULT", "ERROR"}});

    }
};

class NodeFSM : public rclcpp::Node {
public:
    NodeFSM() : rclcpp::Node("demo_node") {}
    CircleFSM my_fsm;
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