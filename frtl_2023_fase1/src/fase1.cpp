#include "fase1/landing_state.hpp"
#include "fase1/initial_takeoff_state.hpp"
#include "fase1/takeoff_state.hpp"
#include "fase1/visit_base_state.hpp"
#include "fase1/return_home_state.hpp"
#include "fase1/finding_bases_state.hpp"
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <iostream>


struct Base{
    cv::Point3f coordinates;
    bool found;
    bool visited;
}

class Fase1FSM : public fsm::FSM {
public:
    Fase1FSM() : fsm::FSM({"ERROR", "FINISHED"}) {

        // Matriz de Parametros da Camera
        double cam_data[][3] = {{524.426799, 0.000000, 306.771757},
                        {0.000000, 526.930449, 193.839579},
                        {0.000000,0.000000, 1.000000}};
        cv::Mat cam_matrix(3, 3, CV_64F, cam_data);

        // Vetores de pontos da cruz
        std::vector<cv::Point3f> objectPoints;
        objectPoints.push_back(cv::Point3f(0, 0, 0));
        objectPoints.push_back(cv::Point3f(4.0f, 5.0f, 6.0f));
        objectPoints.push_back(cv::Point3f(7.0f, 8.0f, 9.0f));

        // Declaracao das bases
        std::vector<Base> bases;
        bases.push_back({{0, 0, 0}, true, true});
        for(int i=0; i<5; i++){
            bases.push_back({{0, 0, 0}, false, false});
        }
        
        // BLACKBOARD
        this->blackboard_set<Drone>("drone", new Drone());
        this->blackboard_set<cv::Mat>("Camera Parameters", cam_matrix);
        // TIME LIMIT = 30s aqui
        this->blackboard_set<double>("time limit", 30);
        this->blackboard_set<std::vector<Base>>("Bases", bases);



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


