#include "fase1/drone.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <iostream>

#include <iostream>
#include <string>
#include "cv_utils/videoTest.hpp"
#include "cv_utils/ColorBoxFilter.hpp"
#include "cv_utils/ContourExtractor.hpp"
// remover elipses concentricas?
#include "cv_utils/EllipseFitter.hpp"
#include "cv_utils/PolygonIdentifier.hpp"
#include "cv_utils/PointCloudTracker.hpp"
#include "cv_utils/ImageToWorldConverter.hpp"
#include <opencv2/core.hpp>
// transformar displayable result em arquivo à parte
/*
landing pad 1: 31, 90, 88, 62, 223, 255 ou 27, 61, 101, 75, 255, 255
landing pad 2, 3, 4, 5: 17, 49, 131, 96, 255, 255
*/
ColorBoxFilter yellowFilter(27, 61, 101, 75, 255, 255);
ContourExtractor extractor(100);
EllipseFitter fitter(1500);
PolygonIdentifier plusId(12, 1, false, 1.0 / 16.0);
PolygonIdentifier rectId(4, 1, true, 1.0 / 15.0);
PointCloudTracker tracker(0.9, 5, 50);

/*
class node : rclcpp::Node() 
public:
    execute() {

        auto cv_br = drone.getVerticalCamera()

        extract_edges(out, cv_br.image)

    }
*/

class NodeFSM : public rclcpp::Node {
public:
    Drone drone;
    NodeFSM() : rclcpp::Node("demo_node") {
        drone.create_image_publisher("/my_image");

    }
    execute(){
        auto extrac_img = drone.getVerticalImage();
        drone.publish_image("/my_image", yellowFilter(extrac_img->image) )
        //cv::imwrite("yellowboxfilter.jpg", yellowFilter(extrac_img->image));
        
    }
};

int main(int argc, const char * argv[]){
    rclcpp::init(argc,argv);
    ~
    auto my_node = std::make_shared<NodeFSM>();
    while (rclcpp::ok()) {
        my_node.execute();
        rclcpp::spin_some(my_node);
    }

    rclcpp::shutdown();
    
    return 0;
    //cv::Mat rgb = cv::imread("../../fase1/marco_pouso.jpg");
    //cv::imwrite("../../fase1/yellowboxfilter.jpg", yellowFilter(rgb));
}