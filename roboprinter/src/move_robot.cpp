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
const double x_scale = -0.01;
const double y_scale = 0.01;

const double z_drawing = 0.01;
const double z_movement = 0.1;

geometry_msgs::msg::Pose new_pose(double& x, double& y, bool is_drawing) {
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1;
    pose.position.x = x_scale * x + x_offset;
    pose.position.y = y_scale * y + y_offset;
    pose.position.z = is_drawing ? z_drawing : z_movement;

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
    auto const logger = rclcpp::get_logger("roboprinter");
    RCLCPP_INFO(logger, "Startup");

    // Load .svg file
    tinyxml2::XMLDocument doc;
    doc.LoadFile("/home/ergoash/roboprinter_ws/src/drawing.svg");
    RCLCPP_INFO(logger, "TinyXML output after loading file: %s", doc.ErrorStr()); 

    std::vector<geometry_msgs::msg::Pose> poses;
    tinyxml2::XMLElement* current_element = doc.FirstChildElement("svg")->FirstChildElement("g")->FirstChildElement();
    do {
        // TODO add support for other elements than "path"
        // TODO add support for splitting values using " " alongside ","

        // get "d" attribuute
        const char* path_cstr = "null";
        current_element->QueryStringAttribute("d", &path_cstr);

        // split "d" attribute content into substrings by spaces
        const std::string new_str = path_cstr;
        std::vector<std::string> path = rcpputils::split(new_str, ' ', true); // skip_empty = true

        // prepare poses depending on a given task
        bool is_in_absolute_state = false;
        for (std::size_t i = 0; i < path.size(); i++) {
            double x = 0; // cached x value
            double y = 0; // cached y value
            RCLCPP_INFO(logger, "XML attr: %s", path[i].c_str());
    
            if (path[i] == "M") {
                auto substring = rcpputils::split(path[i+1], ',', true);

                x = std::stod(substring[0]);
                y = std::stod(substring[1]);
                is_in_absolute_state = true;
    
                poses.push_back(new_pose(x, y, false));
                poses.push_back(new_pose(x, y, true));
    
                i++; // skip one iteration
                continue;
            }
    
            if (path[i] == "m") {
                auto substring = rcpputils::split(path[i+1], ',', true);

                x += std::stod(substring[0]);
                y += std::stod(substring[1]);
                is_in_absolute_state = false;
    
                poses.push_back(new_pose(x, y, false));
                poses.push_back(new_pose(x, y, true));
    
                i++; // skip one iteration
                continue;
            }
    
            if (path[i] == "H") {
                x = std::stod(path[i+1]);
    
                poses.push_back(new_pose(x, y, true));
                i++; // skip one iteration
                continue;
            }
    
            if (path[i] == "h") {
                x += std::stod(path[i+1]);
    
                poses.push_back(new_pose(x, y, true));
                i++; // skip one iteration
                continue;
            }
    
            if (path[i] == "V") {
                y = std::stod(path[i+1]);
    
                poses.push_back(new_pose(x, y, true));
                i++; // skip one iteration
                continue;
            }
    
            if (path[i] == "v") {
                y += std::stod(path[i+1]);
    
                poses.push_back(new_pose(x, y, true));
                i++; // skip one iteration
                continue;
            }
    
            if (path[i] == "L") {
                auto substring = rcpputils::split(path[i+1], ',', true);
                x = std::stod(substring[0]);
                y = std::stod(substring[1]);
    
                poses.push_back(new_pose(x, y, true));
                i++; // skip one iteration
                continue;
            }
    
            if (path[i] == "l") {
                auto substring = rcpputils::split(path[i+1], ',', true);
                x += std::stod(substring[0]);
                y += std::stod(substring[1]);
    
                poses.push_back(new_pose(x, y, true));
                i++; // skip one iteration
                continue;
            }
    
            // Equals to "L" or "l" command
            if (isdigit(path[i][0])) {
                auto substring = rcpputils::split(path[i], ',', true);
                // Type depends on previous "M"/"m" command
                if (is_in_absolute_state) {
                    x = std::stod(substring[0]);
                    y = std::stod(substring[1]);
                } else {
                    x += std::stod(substring[0]);
                    y += std::stod(substring[1]);
                }
    
                poses.push_back(new_pose(x, y, true));
                // Do not skip one iteration
                continue;
            }
    
            continue;
        }

        // Find new sibling for next iteration
        current_element = current_element->NextSiblingElement();
    } while (current_element != nullptr);
    RCLCPP_INFO(logger, "Created %d poses", (int) poses.size());

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "panda_arm"); // TODO roboprinter

    RCLCPP_INFO(logger, "Begin setting targets...");
    move_group_interface.setPoseTargets(poses);

    // Create a plan to that target pose
    RCLCPP_INFO(logger, "Begin planning...");
    auto const [success, plan] = [&move_group_interface] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if(success) {
        RCLCPP_INFO(logger, "Planning succeeded!");
        move_group_interface.execute(plan);
        RCLCPP_INFO(logger, "Plan executed!");
    } else {
        RCLCPP_ERROR(logger, "Planning failed!");
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
