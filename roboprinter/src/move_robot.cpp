#include <cstdio>
#include <memory>
#include <tinyxml2.h>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

using namespace tinyxml2;

int main(int argc, char * argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
            "move_robot",
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("roboprinter-logger");
    RCLCPP_INFO(logger, "Startup");

    XMLDocument doc;
    doc.LoadFile("/home/ergoash/roboprinter_ws/src/drawing.svg");
    RCLCPP_INFO(logger, "TinyXML output: %s", doc.ErrorStr()); 
 
    const char* path = "No path found";
    doc.FirstChildElement("svg")->FirstChildElement("g")->FirstChildElement("path")->QueryStringAttribute("d", &path);
    RCLCPP_INFO(logger, "Path: %s", path);

    // Create the MoveIt MoveGroup Interface
    //using moveit::planning_interface::MoveGroupInterface;
    //auto move_group_interface = MoveGroupInterface(node, "panda_arm");

    // Set a target Pose
    auto const target_pose = []{
        geometry_msgs::msg::Pose msg;
        msg.orientation.w = 1.0;
        msg.position.x = 0.28;
        msg.position.y = -0.2;
        msg.position.z = 0.5;
        return msg;
    }();
    //move_group_interface.setPoseTarget(target_pose);

    // Create a plan to that target pose
    /*
    auto const [success, plan] = [&move_group_interface]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if(success) {
        move_group_interface.execute(plan);
    } else {
        RCLCPP_ERROR(logger, "Planning failed!");
    }
    */

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
