#include <cstdio>
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <ctype.h>
#include <math.h>

#include <tinyxml2.h>

#include <rclcpp/rclcpp.hpp>
#include <rcpputils/split.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

static const rclcpp::Logger logger = rclcpp::get_logger("roboprinter");

//0.09
const double x_offset = 0.15;     // Paper sheet is located 9 cm from robot space origin
const double y_offset = 0.279/2;  // Half height of a A4 paper sheet
const double x_scale = 0.001;     // mm to m conversion
const double y_scale = -0.001;    // mm to m conversion

const double z_drawing = 0.001;   // 1 mm distance from paper
const double z_movement = 0.03;   // 30 mm distance from paper

enum class path_node_type {
	beginning,
	normal,
	end
};

bool generate_cartesian_plan(
		moveit::planning_interface::MoveGroupInterface& move_group_interface, 
		std::vector<geometry_msgs::msg::Pose>& poses,
		moveit_visual_tools::MoveItVisualTools visual_tools
) {
	auto ok = true;
	for (auto pose : poses) {
		move_group_interface.setPoseTarget(pose);
		moveit::planning_interface::MoveGroupInterface::Plan result;
		ok = static_cast<bool>(move_group_interface.plan(result));
		if (ok == false) break;
		move_group_interface.execute(result);

		//visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue planning");
	}
	//moveit_msgs::msgs::RobotTrajectory trajectory;//(move_group_interface.getRobotModel());
	//const double jump_threshold = 0.0;
	//const double eef_step = 0.01;
	//double fraction = move_group_interface.computeCartesianPath(poses, eef_step, jump_threshold, trajectory);

	if (ok) {
		RCLCPP_INFO(logger, "Planning succeeded!");
		//move_group_interface.execute(trajectory);
		RCLCPP_INFO(logger, "Plan executed!");
	} else {
		RCLCPP_ERROR(logger, "Planning failed!");
	}
	return ok;
}

bool generate_joint_plan(
		moveit::planning_interface::MoveGroupInterface& move_group_interface, 
		std::vector<geometry_msgs::msg::Pose>& poses
) {
	move_group_interface.setPoseTarget(poses[1]);
	moveit::planning_interface::MoveGroupInterface::Plan result;
	auto const ok = static_cast<bool>(move_group_interface.plan(result));
	if (ok) {
		RCLCPP_INFO(logger, "Planning succeeded!");
		move_group_interface.execute(result);
		RCLCPP_INFO(logger, "Plan executed!");
	} else {
		RCLCPP_ERROR(logger, "Planning failed!");
	}
	return ok;
}

std::pair<geometry_msgs::msg::Pose, path_node_type> new_pose(double& x, double& y, path_node_type type) {
	geometry_msgs::msg::Pose pose;
	pose.orientation.x = -0.707;
	pose.orientation.y = 0.707;
	pose.orientation.z = 0;
	pose.orientation.w = 0;
	pose.position.x = x_scale * x + x_offset;
	pose.position.y = y_scale * y + y_offset;

	switch (type) {
		case path_node_type::normal:
			pose.position.z = z_drawing;
			break;
		case path_node_type::beginning:
		case path_node_type::end:
		default:
			pose.position.z = z_movement;
	}

	return std::make_pair(pose, type);
}

int main(int argc, char * argv[])
{
	// Initialize ROS and create a ROS Node
	rclcpp::init(argc, argv);
	auto const node = std::make_shared<rclcpp::Node>("move_robot",
			rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
			);
	RCLCPP_INFO(logger, "Startup");

	// Load .svg file
	tinyxml2::XMLDocument doc;
	doc.LoadFile("/home/roboprinter/roboprinter_ws/src/drawing.svg");
	RCLCPP_INFO(logger, "TinyXML output after loading file: %s", doc.ErrorStr()); 

	std::vector<std::pair<geometry_msgs::msg::Pose, path_node_type>> poses;
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
		double x = 0; // cached x value
		double y = 0; // cached y value
		for (std::size_t i = 0; i < path.size(); i++) {
			if (path[i] == "M") {
				auto substring = rcpputils::split(path[i+1], ',', true);

				x = std::stod(substring[0]);
				y = std::stod(substring[1]);
				is_in_absolute_state = true;

				poses.push_back(new_pose(x, y, path_node_type::beginning));
				poses.push_back(new_pose(x, y, path_node_type::normal));
				// TODO execute only one push_back for the same path for multiple M/m commands

				i++; // skip one iteration
				continue;
			}

			if (path[i] == "m") {
				auto substring = rcpputils::split(path[i+1], ',', true);

				x += std::stod(substring[0]);
				y += std::stod(substring[1]);
				is_in_absolute_state = false;

				poses.push_back(new_pose(x, y, path_node_type::beginning));
				poses.push_back(new_pose(x, y, path_node_type::normal));

				i++; // skip one iteration
				continue;
			}

			if (path[i] == "H") {
				x = std::stod(path[i+1]);

				poses.push_back(new_pose(x, y, path_node_type::normal));
				i++; // skip one iteration
				continue;
			}

			if (path[i] == "h") {
				x += std::stod(path[i+1]);

				poses.push_back(new_pose(x, y, path_node_type::normal));
				i++; // skip one iteration
				continue;
			}

			if (path[i] == "V") {
				y = std::stod(path[i+1]);

				poses.push_back(new_pose(x, y, path_node_type::normal));
				i++; // skip one iteration
				continue;
			}

			if (path[i] == "v") {
				y += std::stod(path[i+1]);

				poses.push_back(new_pose(x, y, path_node_type::normal));
				i++; // skip one iteration
				continue;
			}

			if (path[i] == "L") {
				auto substring = rcpputils::split(path[i+1], ',', true);
				x = std::stod(substring[0]);
				y = std::stod(substring[1]);

				poses.push_back(new_pose(x, y, path_node_type::normal));
				i++; // skip one iteration
				continue;
			}

			if (path[i] == "l") {
				auto substring = rcpputils::split(path[i+1], ',', true);
				x += std::stod(substring[0]);
				y += std::stod(substring[1]);

				poses.push_back(new_pose(x, y, path_node_type::normal));
				i++; // skip one iteration
				continue;
			}

			// Equals to "L" or "l" command
			if (isdigit(path[i][0]) || path[i][0] == '-') {
				auto substring = rcpputils::split(path[i], ',', true);
				// Type depends on previous "M"/"m" command
				if (is_in_absolute_state) {
					x = std::stod(substring[0]);
					y = std::stod(substring[1]);
				} else {
					x += std::stod(substring[0]);
					y += std::stod(substring[1]);
				}

				poses.push_back(new_pose(x, y, path_node_type::normal));
				// Do not skip one iteration
				continue;
			}

			continue;
		}

		// Add last position above that will be a start of a transition to the next path
		poses.push_back(new_pose(x, y, path_node_type::end));

		// Find new sibling for next iteration
		current_element = current_element->NextSiblingElement();
	} while (current_element != nullptr);

	// Create the MoveIt MoveGroup Interface
	using moveit::planning_interface::MoveGroupInterface;
	auto move_group_interface = MoveGroupInterface(node, "main_group");

	// Create MoveIt Visual Tools 
	namespace rvt = rviz_visual_tools;
	moveit_visual_tools::MoveItVisualTools visual_tools(node, "base_link", "roboprinter_visuals",
			move_group_interface.getRobotModel());

	visual_tools.deleteAllMarkers(); 
	visual_tools.loadRemoteControl();
	visual_tools.trigger();

	RCLCPP_INFO(logger, "Created %d poses", (int) poses.size());
	std::vector<geometry_msgs::msg::Pose> path;
	for (size_t i = 0; i < poses.size(); i++) {
		auto pose = poses[i].first;
		auto color = rviz_visual_tools::GREEN;
		path.push_back(pose);
		switch (poses[i].second) {
			case path_node_type::beginning:
				color = rviz_visual_tools::YELLOW;
				break;
			case path_node_type::normal:
				color = rviz_visual_tools::GREEN;
				break;
			case path_node_type::end:
				color = rviz_visual_tools::BLUE;
				break;
			default:
				color = rviz_visual_tools::GREEN;
		}
		visual_tools.publishSphere(pose, color, rviz_visual_tools::MEDIUM);
		RCLCPP_INFO(logger, "Pose %d pos: %f, %f, %f", (int) i, pose.position.x, pose.position.y, pose.position.z);
	}
	visual_tools.publishPath(path, rviz_visual_tools::RED, rviz_visual_tools::XXSMALL);
	visual_tools.trigger();

	//visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start planning");

	/*
	// Setup constraints
	moveit_msgs::msg::OrientationConstraint ocm;
	ocm.link_name = "end_effector";
	ocm.header.frame_id = "base_link";
	ocm.orientation.x = -0.707;
	ocm.orientation.y = 0.707;
	ocm.orientation.z = 0;
	ocm.orientation.w = 0;
	ocm.absolute_x_axis_tolerance = 0.1;
	ocm.absolute_y_axis_tolerance = 0.1;
	ocm.absolute_z_axis_tolerance = 0.1;
	ocm.weight = 1.0;

	// Now, set it as the path constraint for the group.
	moveit_msgs::msg::Constraints constraints;
	constraints.orientation_constraints.push_back(ocm);
	move_group_interface.setPathConstraints(constraints);
	*/

	RCLCPP_INFO(logger, "Begin setting targets...");
	std::vector<geometry_msgs::msg::Pose> current_path;
	for (size_t i = 0; i < poses.size(); i++) {
		auto pose = poses[i].first;
		auto type = poses[i].second;
		bool success;
		switch (type) {
			default:
				break;
			case path_node_type::beginning:
			case path_node_type::normal:
				current_path.push_back(pose);
				break;
			case path_node_type::end:
				current_path.push_back(pose);

				// Generate path on paper
				RCLCPP_INFO(logger, "Begin cartesian planning...");
				success = generate_cartesian_plan(move_group_interface, current_path, visual_tools);
				current_path.clear();
				if (success == false)
					goto loop_end; // TODO message

				// Omit if it is the last point
				if (i == poses.size() + 1) // TODO go back to home position
					goto loop_end; // TODO message



				// Create joint-based path between the end 
				current_path.push_back(pose);
				current_path.push_back(poses[i+1].first);
				i++; // skip one iteration

				// Generate path in the air
				RCLCPP_INFO(logger, "Begin joint planning...");
				success = generate_joint_plan(move_group_interface, current_path);
				current_path.clear();
				if (success == false)
					goto loop_end; // TODO message

				break;
		}
	}
loop_end:

	rclcpp::shutdown();
	return 0;
}
