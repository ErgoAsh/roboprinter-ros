#include <cstdio>
#include <memory>
#include <string>
#include <vector>
#include <ctype.h>

#include <tinyxml2.h>

#include <rclcpp/rclcpp.hpp>
#include <rcpputils/split.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

const double x_offset = 0;
const double y_offset = 0;
const double x_scale = -1;
const double y_scale = 1;

const double z_drawing = 1;
const double z_movement = 10;

geometry_msgs::msg:Pose new_pose(double& x, double& y, bool is_drawing) const {
    geometry_msgs::msg::Pose pose;
    msg.orientation.w = 1;
    msg.position.x = x_scale * x + x_offset;
    msg.position.y = y_scale * y + y_offset;
    msg.position.z = is_drawing ? z_drawing, z_movement;

    return pose;
}

int main(int argc, char * argv[])
{
    // Initialize ROS and create a ROS Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>("move_robot",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("roboprinter-logger");
    RCLCPP_INFO(logger, "Startup");

    // Load .svg file
    tinyxml2::XMLDocument doc;
    doc.LoadFile("/home/ergoash/roboprinter_ws/src/drawing.svg");
    RCLCPP_INFO(logger, "TinyXML output after loading file: %s", doc.ErrorStr()); 

    std::vector<geometry::msg::Pose> poses;
    tinyxml2::XMLElement current_element = doc.FirstChildElement("svg")->FirstChildElement("g").FirstChildElement();
    do {
        // TODO add support for other elements than "path"
        // TODO add support for splitting values using " " alongside ","

        // get "d" attribuute
        const char* path_cstr = "null";
        current_element->querystringattribute("d", &path_cstr);

        // split "d" attribute content into substrings by spaces
        const std::string new_str = path_cstr;
        std::vector<std::string> path = rcpputils::split(new_path, " ", skip_empty=true);

        // prepare poses depending on a given task
        bool is_in_absolute_state = false;
        for (std::size_t i = 0; i < output.size(); i++) {
            double x = 0; // cached x value
            double y = 0; // cached y value
    
            if (path[i] == "m") {
                auto substring = rcpputils::split(path[i+i], ",", skip_empty=true);
                x = std::stod(substring[0]);
                y = std::stod(substring[1]);
                is_in_absolute_state = true;
    
                poses.push_back(new_pose(x, y, false));
                poses.push_back(new_pose(x, y, true));
    
                i++; // skip one iteration
                continue;
            }
    
            if (path[i] == "m") {
                auto substring = rcpputils::split(path[i+i], ",", skip_empty=true);
                x += std::stod(substring[0]);
                y += std::stod(substring[1]);
                is_in_absolute_state = false;
    
                poses.push_back(new_pose(x, y, false));
                poses.push_back(new_pose(x, y, true));
    
                i++; // skip one iteration
                continue;
            }
    
            if (path[i] == "h") {
                x = std::stod(path[i+1]);
    
                poses.push_back(x, y, true);
                i++; // skip one iteration
                continue;
            }
    
            if (path[i] == "h") {
                x += std::stod(path[i+1]);
    
                poses.push_back(x, y, true);
                i++; // skip one iteration
                continue;
            }
    
            if (path[i] == "v") {
                y = std::stod(path[i+1]);
    
                poses.push_back(x, y, true);
                i++; // skip one iteration
                continue;
            }
    
            if (path[i] == "v") {
                y += std::stod(path[i+1]);
    
                poses.push_back(x, y, true);
                i++; // skip one iteration
                continue;
            }
    
            if (path[i] == "l") {
                auto substring = rcpputils::split(path[i+i], ",", skip_empty=true);
                x = std::stod(substring[0]);
                y = std::stod(substring[1]);
    
                poses.push_back(x, y, true);
                i++; // skip one iteration
                continue;
            }
    
            if (path[i] == "l") {
                auto substring = rcpputils::split(path[i+i], ",", skip_empty=true);
                x += std::stod(substring[0]);
                y += std::stod(substring[1]);
    
                poses.push_back(x, y, true);
                i++; // skip one iteration
                continue;
            }
    
            // Equals to "L" or "l" command
            if (isdigit(path[i][0])) {
                auto substring = rcpputils::split(path[i+i], ",", skip_empty=true);
                // Type depends on previous "M"/"m" command
                if (is_in_absolute_state) {
                    x = std::stod(substring[0]);
                    y = std::stod(substring[1]);
                } else {
                    x += std::stod(substring[0]);
                    y += std::stod(substring[1]);
                }
    
                poses.push_back(new_pose(x, y, true));
                // Do not skip iteration
                continue;
            }
    
            continue;
        }

        // Find new sibling for next iteration
        current_element = current_element.nextsiblingelement();
    } while (current_element != nullptr);

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "panda_arm"); // TODO roboprinter

    // Set a target Pose
    auto const target_pose = []{
        geometry_msgs::msg::Pose msg;
        msg.orientation.w = 1.0;
        msg.position.x = 0.28;
        msg.position.y = -0.2;
        msg.position.z = 0.5;
        return msg;
    }();
    move_group_interface.setPoseTarget(target_pose);

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
