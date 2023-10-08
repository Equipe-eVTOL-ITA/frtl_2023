#include "fase1_safe/takeoff_state.hpp"
#include "fase1_safe/return_home_state.hpp"
#include "fase1_safe/visit_base_state.hpp"
#include "fase1_safe/landing_state.hpp"
#include "fase1_safe/base.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <iostream>

class Fase1SafeFSM : public fsm::FSM {
public:
    Fase1SafeFSM() : fsm::FSM({"ERROR", "FINISHED"}) {

        // Declaracao das bases
        std::vector<Base> bases;
        bases.push_back({{1.0f, 3.5f, -1.5f}, false});
        bases.push_back({{7.0f, 1.0f, -1.5f}, false});
        
        // BLACKBOARD
        this->blackboard_set<Drone>("drone", new Drone());
        this->blackboard_set<float>("height", -2.5f);
        this->blackboard_set<std::vector<Base>>("Bases", bases);

        Drone* drone = blackboard_get<Drone>("drone");
        drone->create_image_publisher("/transformed_vertical_image");
        drone->create_image_publisher("/transformed_vertical_image");

        this->add_state("TAKEOFF", std::make_unique<TakeoffState>());
        this->add_state("RETURN HOME", std::make_unique<ReturnHomeState>());
        this->add_state("VISIT BASE", std::make_unique<VisitBaseState>());
        this->add_state("LAND", std::make_unique<LandingState>());

        this->add_transitions("TAKEOFF", {{"VISIT NEXT BASE", "VISIT BASE"},{"SEG FAULT", "ERROR"}});
        this->add_transitions("TAKEOFF", {{"FINISHED BASES", "RETURN HOME"},{"SEG FAULT", "ERROR"}});

        this->add_transitions("VISIT BASE", {{"BASE VISITED", "LAND"},{"SEG FAULT", "ERROR"}});
        this->add_transitions("LAND", {{"LANDED", "TAKEOFF"},{"SEG FAULT", "ERROR"}});
        this->add_transitions("RETURN HOME", {{"RETURNED HOME", "FINISHED"},{"SEG FAULT", "ERROR"}});

    }
};

class NodeFSM : public rclcpp::Node {
public:
    NodeFSM() : rclcpp::Node("fase1_node") {}
    Fase1SafeFSM my_fsm;
};

int main(int argc, const char * argv[]){
    rclcpp::init(argc,argv);
    std::cout << "Iniciou.";
    auto my_node = std::make_shared<NodeFSM>();
    while (rclcpp::ok() && !my_node->my_fsm.is_finished()) {
        my_node->my_fsm.execute();
        rclcpp::spin_some(my_node);
    }

    std::cout << my_node->my_fsm.get_fsm_outcome() << std::endl;
    rclcpp::shutdown();
    
    return 0;
}

