/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2016 \n
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 * Project name: Care-O-bot
 * \note
 * ROS stack name: autopnp
 * \note
 * ROS package name: ipa_room_exploration
 *
 * \author
 * Author: Florian Jordan, Richard Bormann
 * \author
 * Supervised by: Richard Bormann
 *
 * \date Date of creation: 03.2016
 *
 * \brief
 *
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer. \n
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution. \n
 * - Neither the name of the Fraunhofer Institute for Manufacturing
 * Engineering and Automation (IPA) nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/


#include <ipa_room_exploration/room_exploration_action_server.h>

// constructor
RoomExplorationServer::RoomExplorationServer(ros::NodeHandle nh, std::string name_of_the_action) :
	node_handle_(nh),
	room_exploration_server_(node_handle_, name_of_the_action, boost::bind(&RoomExplorationServer::exploreRoom, this, _1), false)
{
	// dynamic reconfigure
	room_exploration_dynamic_reconfigure_server_.setCallback(boost::bind(&RoomExplorationServer::dynamic_reconfigure_callback, this, _1, _2));

	// Parameters
	std::cout << "\n--------------------------\nRoom Exploration Parameters:\n--------------------------\n";
	node_handle_.param("room_exploration_algorithm", room_exploration_algorithm_, 1);
	std::cout << "room_exploration/room_exploration_algorithm = " << room_exploration_algorithm_ << std::endl;
	node_handle_.param("display_trajectory", display_trajectory_, false);
	std::cout << "room_exploration/display_trajectory = " << display_trajectory_ << std::endl;

	node_handle_.param("map_correction_closing_neighborhood_size", map_correction_closing_neighborhood_size_, 2);
	std::cout << "room_exploration/map_correction_closing_neighborhood_size = " << map_correction_closing_neighborhood_size_ << std::endl;

	node_handle_.param("return_path", return_path_, true);
	std::cout << "room_exploration/return_path = " << return_path_ << std::endl;
	node_handle_.param("execute_path", execute_path_, false);
	std::cout << "room_exploration/execute_path = " << execute_path_ << std::endl;
	node_handle_.param("goal_eps", goal_eps_, 0.35);
	std::cout << "room_exploration/goal_eps = " << goal_eps_ << std::endl;
	node_handle_.param("use_dyn_goal_eps", use_dyn_goal_eps_, false);
	std::cout << "room_exploration/use_dyn_goal_eps = " << use_dyn_goal_eps_ << std::endl;
	node_handle_.param("interrupt_navigation_publishing", interrupt_navigation_publishing_, false);
	std::cout << "room_exploration/interrupt_navigation_publishing = " << interrupt_navigation_publishing_ << std::endl;
	node_handle_.param("revisit_areas", revisit_areas_, false);
	std::cout << "room_exploration/revisit_areas = " << revisit_areas_ << std::endl;
	node_handle_.param("left_sections_min_area", left_sections_min_area_, 0.01);
	std::cout << "room_exploration/left_sections_min_area_ = " << left_sections_min_area_ << std::endl;
	global_costmap_topic_ = "/move_base/global_costmap/costmap";
	node_handle_.param<std::string>("global_costmap_topic", global_costmap_topic_);
	std::cout << "room_exploration/global_costmap_topic = " << global_costmap_topic_ << std::endl;
	node_handle_.param<std::string>("coverage_check_service_name", coverage_check_service_name_, "/room_exploration/coverage_check_server/coverage_check");
	std::cout << "room_exploration/coverage_check_service_name = " << coverage_check_service_name_ << std::endl;
	map_frame_ = "map";
	node_handle_.param<std::string>("map_frame", map_frame_);
	std::cout << "room_exploration/map_frame = " << map_frame_ << std::endl;
	camera_frame_ = "base_link";
	node_handle_.param<std::string>("camera_frame", camera_frame_);
	std::cout << "room_exploration/camera_frame = " << camera_frame_ << std::endl;

	ROS_INFO("You have chosen the boustrophedon exploration method.");
	
	
		node_handle_.param("min_cell_area", min_cell_area_, 10.0);
		std::cout << "room_exploration/min_cell_area_ = " << min_cell_area_ << std::endl;
		node_handle_.param("path_eps", path_eps_, 2.0);
		std::cout << "room_exploration/path_eps_ = " << path_eps_ << std::endl;
		node_handle_.param("grid_obstacle_offset", grid_obstacle_offset_, 0.0);
		std::cout << "room_exploration/grid_obstacle_offset_ = " << grid_obstacle_offset_ << std::endl;
		node_handle_.param("max_deviation_from_track", max_deviation_from_track_, -1);
		std::cout << "room_exploration/max_deviation_from_track_ = " << max_deviation_from_track_ << std::endl;
		node_handle_.param("cell_visiting_order", cell_visiting_order_, 1);
		std::cout << "room_exploration/cell_visiting_order = " << cell_visiting_order_ << std::endl;

	if (revisit_areas_ == true)
		ROS_INFO("Areas not seen after the initial execution of the path will be revisited.");
	else
		ROS_INFO("Areas not seen after the initial execution of the path will NOT be revisited.");


	// min area for revisiting left sections

	path_pub_ = node_handle_.advertise<nav_msgs::Path>("coverage_path", 2);

	//Start action server
	room_exploration_server_.start();

	ROS_INFO("Action server for room exploration has been initialized......");
}


// Callback function for dynamic reconfigure.
void RoomExplorationServer::dynamic_reconfigure_callback(ipa_room_exploration::RoomExplorationConfig &config, uint32_t level)
{
	// set segmentation algorithm
	std::cout << "######################################################################################" << std::endl;
	std::cout << "Dynamic reconfigure request:" << std::endl;

	room_exploration_algorithm_ = config.room_exploration_algorithm;
	std::cout << "room_exploration/path_planning_algorithm_ = " << room_exploration_algorithm_ << std::endl;

	map_correction_closing_neighborhood_size_ = config.map_correction_closing_neighborhood_size;
	std::cout << "room_exploration/map_correction_closing_neighborhood_size_ = " << map_correction_closing_neighborhood_size_ << std::endl;

	return_path_ = config.return_path;
	std::cout << "room_exploration/return_path_ = " << return_path_ << std::endl;
	execute_path_ = config.execute_path;
	std::cout << "room_exploration/execute_path_ = " << execute_path_ << std::endl;
	goal_eps_ = config.goal_eps;
	std::cout << "room_exploration/goal_eps_ = " << goal_eps_ << std::endl;
	use_dyn_goal_eps_  = config.use_dyn_goal_eps;
	std::cout << "room_exploration/use_dyn_goal_eps_ = " << use_dyn_goal_eps_ << std::endl;
	interrupt_navigation_publishing_ = config.interrupt_navigation_publishing;
	std::cout << "room_exploration/interrupt_navigation_publishing_ = " << interrupt_navigation_publishing_ << std::endl;
	revisit_areas_ = config.revisit_areas;
	std::cout << "room_exploration/revisit_areas_ = " << revisit_areas_ << std::endl;
	left_sections_min_area_ = config.left_sections_min_area;
	std::cout << "room_exploration/left_sections_min_area = " << left_sections_min_area_ << std::endl;
	global_costmap_topic_ = config.global_costmap_topic;
	std::cout << "room_exploration/global_costmap_topic_ = " << global_costmap_topic_ << std::endl;
	coverage_check_service_name_ = config.coverage_check_service_name;
	std::cout << "room_exploration/coverage_check_service_name_ = " << coverage_check_service_name_ << std::endl;
	map_frame_ = config.map_frame;
	std::cout << "room_exploration/map_frame_ = " << map_frame_ << std::endl;
	camera_frame_ = config.camera_frame;
	std::cout << "room_exploration/camera_frame_ = " << camera_frame_ << std::endl;


		min_cell_area_ = config.min_cell_area;
		std::cout << "room_exploration/min_cell_area_ = " << min_cell_area_ << std::endl;
		path_eps_ = config.path_eps;
		std::cout << "room_exploration/path_eps_ = " << path_eps_ << std::endl;
		grid_obstacle_offset_ = config.grid_obstacle_offset;
		std::cout << "room_exploration/grid_obstacle_offset_ = " << grid_obstacle_offset_ << std::endl;
		max_deviation_from_track_ = config.max_deviation_from_track;
		std::cout << "room_exploration/max_deviation_from_track_ = " << max_deviation_from_track_ << std::endl;
		cell_visiting_order_ = config.cell_visiting_order;
		std::cout << "room_exploration/cell_visiting_order = " << cell_visiting_order_ << std::endl;

	if (revisit_areas_ == true)
		std::cout << "Areas not seen after the initial execution of the path will be revisited." << std::endl;
	else
		std::cout << "Areas not seen after the initial execution of the path will NOT be revisited." << std::endl;

	std::cout << "######################################################################################" << std::endl;
}


// Function executed by Call.
void RoomExplorationServer::exploreRoom(const ipa_building_msgs::RoomExplorationGoalConstPtr &goal)
{
	ROS_INFO("*****Room Exploration action server*****");

	// ***************** I. read the given parameters out of the goal *****************
	// todo: this is only correct if the map is not rotated
	const cv::Point2d map_origin(goal->map_origin.position.x, goal->map_origin.position.y);
	const float map_resolution = goal->map_resolution;	// in [m/cell]
	const float map_resolution_inverse = 1./map_resolution;
	std::cout << "map origin: " << map_origin << " m       map resolution: " << map_resolution << " m/cell" << std::endl;

	const float robot_radius = goal->robot_radius;
	const int robot_radius_in_pixel = (robot_radius / map_resolution);
	std::cout << "robot radius: " << robot_radius << " m   (" << robot_radius_in_pixel << " px)" << std::endl;

	const cv::Point starting_position((goal->starting_position.x-map_origin.x)/map_resolution, (goal->starting_position.y-map_origin.y)/map_resolution);
	std::cout << "starting point: (" << goal->starting_position.x << ", " << goal->starting_position.y << ") m   (" << starting_position << " px)" << std::endl;

	planning_mode_ = goal->planning_mode;
	if (planning_mode_==PLAN_FOR_FOOTPRINT)
		std::cout << "planning mode: planning coverage path with robot's footprint" <<std::endl;
	else if (planning_mode_==PLAN_FOR_FOV)
		std::cout << "planning mode: planning coverage path with robot's field of view" <<std::endl;

	// todo: receive map data in nav_msgs::OccupancyGrid format
	// converting the map msg in cv format
	cv_bridge::CvImagePtr cv_ptr_obj;
	cv_ptr_obj = cv_bridge::toCvCopy(goal->input_map, sensor_msgs::image_encodings::MONO8);
	cv::Mat room_map = cv_ptr_obj->image;

	// determine room size
	int area_px = 0;		// room area in pixels
	for (int v=0; v<room_map.rows; ++v)
		for (int u=0; u<room_map.cols; ++u)
			if (room_map.at<uchar>(v,u) >= 250)
				area_px++;
	std::cout << "### room area = " << area_px*map_resolution*map_resolution << " m^2" << std::endl;

	// closing operation to neglect inaccessible areas and map errors/artifacts
	cv::Mat temp;
	cv::erode(room_map, temp, cv::Mat(), cv::Point(-1, -1), map_correction_closing_neighborhood_size_);
	cv::dilate(temp, room_map, cv::Mat(), cv::Point(-1, -1), map_correction_closing_neighborhood_size_);

	// remove unconnected, i.e. inaccessible, parts of the room (i.e. obstructed by furniture), only keep the room with the largest area
	const bool room_not_empty = removeUnconnectedRoomParts(room_map);
	if (room_not_empty == false)
	{
		std::cout << "RoomExplorationServer::exploreRoom: Warning: the requested room is too small for generating exploration trajectories." << std::endl;
		ipa_building_msgs::RoomExplorationResult action_result;
		room_exploration_server_.setAborted(action_result);
		return;
	}

	// get the grid size, to check the areas that should be revisited later
	double grid_spacing_in_meter = 0.0;		// is the square grid cell side length that fits into the circle with the robot's coverage radius or fov coverage radius
	float fitting_circle_radius_in_meter = 0;
	Eigen::Matrix<float, 2, 1> fitting_circle_center_point_in_meter;	// this is also considered the center of the field of view, because around this point the maximum radius incircle can be found that is still inside the fov
	std::vector<Eigen::Matrix<float, 2, 1> > fov_corners_meter(4);
	const double fov_resolution = 1000;		// in [cell/meter]
	if(planning_mode_ == PLAN_FOR_FOV) // read out the given fov-vectors, if needed
	{
		// Get the size of one grid cell s.t. the grid can be completely covered by the field of view (fov) from all rotations around it.
		for(int i = 0; i < 4; ++i)
			fov_corners_meter[i] << goal->field_of_view[i].x, goal->field_of_view[i].y;
		computeFOVCenterAndRadius(fov_corners_meter, fitting_circle_radius_in_meter, fitting_circle_center_point_in_meter, fov_resolution);
		// get the edge length of the grid square that fits into the fitting_circle_radius
		grid_spacing_in_meter = fitting_circle_radius_in_meter*std::sqrt(2);
	}
	else // if planning should be done for the footprint, read out the given coverage radius
	{
		grid_spacing_in_meter = goal->coverage_radius*std::sqrt(2);
	}
	// map the grid size to an int in pixel coordinates, using floor method
	const double grid_spacing_in_pixel = grid_spacing_in_meter/map_resolution;		// is the square grid cell side length that fits into the circle with the robot's coverage radius or fov coverage radius, multiply with sqrt(2) to receive the whole working width
	std::cout << "grid size: " << grid_spacing_in_meter << " m   (" << grid_spacing_in_pixel << " px)" << std::endl;


	// ***************** II. plan the path using the wanted planner *****************
	// todo: consider option to provide the inflated map or the robot radius to the functions instead of inflating with half cell size there
	Eigen::Matrix<float, 2, 1> zero_vector;
	zero_vector << 0, 0;
	std::vector<geometry_msgs::Pose2D> exploration_path;
		// plan path
		if(planning_mode_ == PLAN_FOR_FOV)
			boustrophedon_explorer_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, grid_spacing_in_pixel, grid_obstacle_offset_, path_eps_, cell_visiting_order_, false, fitting_circle_center_point_in_meter, min_cell_area_, max_deviation_from_track_);
		else
			boustrophedon_explorer_.getExplorationPath(room_map, exploration_path, map_resolution, starting_position, map_origin, grid_spacing_in_pixel, grid_obstacle_offset_, path_eps_, cell_visiting_order_, true, zero_vector, min_cell_area_, max_deviation_from_track_);

	// display finally planned path
	if (display_trajectory_ == true)
	{
		std::cout << "printing path" << std::endl;
		cv::Mat fov_path_map;
		for(size_t step=1; step<exploration_path.size(); ++step)
		{
			fov_path_map = room_map.clone();
			cv::resize(fov_path_map, fov_path_map, cv::Size(), 2, 2, cv::INTER_LINEAR);
			if (exploration_path.size() > 0)
#if CV_MAJOR_VERSION<=3
				cv::circle(fov_path_map, 2*cv::Point((exploration_path[0].x-map_origin.x)/map_resolution, (exploration_path[0].y-map_origin.y)/map_resolution), 2, cv::Scalar(150), CV_FILLED);
#else
				cv::circle(fov_path_map, 2*cv::Point((exploration_path[0].x-map_origin.x)/map_resolution, (exploration_path[0].y-map_origin.y)/map_resolution), 2, cv::Scalar(150), cv::FILLED);
#endif
			for(size_t i=1; i<=step; ++i)
			{
				cv::Point p1((exploration_path[i-1].x-map_origin.x)/map_resolution, (exploration_path[i-1].y-map_origin.y)/map_resolution);
				cv::Point p2((exploration_path[i].x-map_origin.x)/map_resolution, (exploration_path[i].y-map_origin.y)/map_resolution);
#if CV_MAJOR_VERSION<=3
				cv::circle(fov_path_map, 2*p2, 2, cv::Scalar(200), CV_FILLED);
#else
				cv::circle(fov_path_map, 2*p2, 2, cv::Scalar(200), cv::FILLED);
#endif
				cv::line(fov_path_map, 2*p1, 2*p2, cv::Scalar(150), 1);
				cv::Point p3(p2.x+5*cos(exploration_path[i].theta), p2.y+5*sin(exploration_path[i].theta));
				if (i==step)
				{
#if CV_MAJOR_VERSION<=3
					cv::circle(fov_path_map, 2*p2, 2, cv::Scalar(80), CV_FILLED);
#else
					cv::circle(fov_path_map, 2*p2, 2, cv::Scalar(80), cv::FILLED);
#endif
					cv::line(fov_path_map, 2*p1, 2*p2, cv::Scalar(150), 1);
					cv::line(fov_path_map, 2*p2, 2*p3, cv::Scalar(50), 1);
				}
			}
//			cv::imshow("cell path", fov_path_map);
//			cv::waitKey();
		}
		// cv::imshow("cell path", fov_path_map);
		cv::waitKey();
	}
	
	ROS_INFO("Room exploration planning finished.");

	ipa_building_msgs::RoomExplorationResult action_result;
	// check if the size of the exploration path is larger then zero
	if(exploration_path.size()==0)
	{
		room_exploration_server_.setAborted(action_result);
		return;
	}

	// if wanted, return the path as the result
	if(return_path_ == true)
	{
		action_result.coverage_path = exploration_path;
		// return path in PoseStamped format as well (e.g. necessary for move_base commands)
		std::vector<geometry_msgs::PoseStamped> exploration_path_pose_stamped(exploration_path.size());
		std_msgs::Header header;
		header.stamp = ros::Time::now();
		header.frame_id = "/map";
		for (size_t i=0; i<exploration_path.size(); ++i)
		{
			exploration_path_pose_stamped[i].header = header;
			exploration_path_pose_stamped[i].header.seq = i;
			exploration_path_pose_stamped[i].pose.position.x = exploration_path[i].x;
			exploration_path_pose_stamped[i].pose.position.y = exploration_path[i].y;
			exploration_path_pose_stamped[i].pose.position.z = 0.;
			Eigen::Quaterniond quaternion;
			quaternion = Eigen::AngleAxisd((double)exploration_path[i].theta, Eigen::Vector3d::UnitZ());
			tf::quaternionEigenToMsg(quaternion, exploration_path_pose_stamped[i].pose.orientation);
		}
		action_result.coverage_path_pose_stamped = exploration_path_pose_stamped;

		nav_msgs::Path coverage_path;
		coverage_path.header.frame_id = "map";
		coverage_path.header.stamp = ros::Time::now();
		coverage_path.poses = exploration_path_pose_stamped;
		path_pub_.publish(coverage_path);
	}	

	room_exploration_server_.setSucceeded(action_result);

	return;
}

	// remove unconnected, i.e. inaccessible, parts of the room (i.e. obstructed by furniture), only keep the room with the largest area
bool RoomExplorationServer::removeUnconnectedRoomParts(cv::Mat& room_map)
{
	// create new map with segments labeled by increasing labels from 1,2,3,...
	cv::Mat room_map_int(room_map.rows, room_map.cols, CV_32SC1);
	for (int v=0; v<room_map.rows; ++v)
	{
		for (int u=0; u<room_map.cols; ++u)
		{
			if (room_map.at<uchar>(v,u) == 255)
				room_map_int.at<int32_t>(v,u) = -100;
			else
				room_map_int.at<int32_t>(v,u) = 0;
		}
	}

	std::map<int, int> area_to_label_map;	// maps area=number of segment pixels (keys) to the respective label (value)
	int label = 1;
	for (int v=0; v<room_map_int.rows; ++v)
	{
		for (int u=0; u<room_map_int.cols; ++u)
		{
			if (room_map_int.at<int32_t>(v,u) == -100)
			{
				const int area = cv::floodFill(room_map_int, cv::Point(u,v), cv::Scalar(label), 0, 0, 0, 8 | cv::FLOODFILL_FIXED_RANGE);
				area_to_label_map[area] = label;
				++label;
			}
		}
	}
	// abort if area_to_label_map.size() is empty
	if (area_to_label_map.size() == 0)
		return false;

	// remove all room pixels from room_map which are not accessible
	const int label_of_biggest_room = area_to_label_map.rbegin()->second;
	std::cout << "label_of_biggest_room=" << label_of_biggest_room << std::endl;
	for (int v=0; v<room_map.rows; ++v)
		for (int u=0; u<room_map.cols; ++u)
			if (room_map_int.at<int32_t>(v,u) != label_of_biggest_room)
				room_map.at<uchar>(v,u) = 0;

	return true;
}


void RoomExplorationServer::downsampleTrajectory(const std::vector<geometry_msgs::Pose2D>& path_uncleaned, std::vector<geometry_msgs::Pose2D>& path, const double min_dist_squared)
{
	// clean path from subsequent double occurrences of the same pose
	path.push_back(path_uncleaned[0]);
	cv::Point last_added_point(path_uncleaned[0].x, path_uncleaned[0].y);
	for (size_t i=1; i<path_uncleaned.size(); ++i)
	{
		const cv::Point current_point(path_uncleaned[i].x, path_uncleaned[i].y);
		cv::Point vector = current_point - last_added_point;
		if (vector.x*vector.x+vector.y*vector.y > min_dist_squared || i==path_uncleaned.size()-1)
		{
			path.push_back(path_uncleaned[i]);
			last_added_point = current_point;
		}
	}
}


// main, initializing server
int main(int argc, char** argv)
{
	ros::init(argc, argv, "room_exploration_server");
	ros::Time::init();

	ros::NodeHandle nh("~");

	RoomExplorationServer explorationObj(nh, ros::this_node::getName());
	ros::spin();

	return 0;
}
