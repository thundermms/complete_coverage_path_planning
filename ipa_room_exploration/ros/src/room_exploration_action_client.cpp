#include <ros/ros.h>
#include <ros/package.h>

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Image.h>
#include <ipa_building_msgs/RoomExplorationAction.h>
#include <ipa_room_exploration/dynamic_reconfigure_client.h>
#include <ipa_room_exploration/timer.h>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include "std_srvs/Trigger.h"


class RoomExplorationClient {
public:
   RoomExplorationClient() : nh(), priv_nh("~"),ac("room_exploration_server", true) {
        ROS_INFO("Waiting for action server to start.");
        ac.waitForServer(); // will wait for infinite time
        ROS_INFO("Action server started");

        // Advertise the auto_path_planning service
        service = nh.advertiseService("/auto_path_planning", &RoomExplorationClient::autoPathPlanningCallback, this);
    }

private:
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh;
    ros::ServiceServer service;
    ros::Subscriber sub_map;
    ros::Subscriber sub_odom;

    actionlib::SimpleActionClient<ipa_building_msgs::RoomExplorationAction> ac;
    sensor_msgs::Image global_labeling;
    nav_msgs::OccupancyGrid::ConstPtr global_occupancy_grid;
    geometry_msgs::Pose2D current_robot_pose;
    bool isDataReady;

    // Callback function for the map subscription
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {   ROS_INFO("getting image");
        global_occupancy_grid = msg;
        cv::Mat map(global_occupancy_grid->info.height, global_occupancy_grid->info.width, CV_8UC1);

        for (int y = 0; y < map.rows; y++)
        {
            for (int x = 0; x < map.cols; x++)
            {
                int8_t occupancy_value = global_occupancy_grid->data[y * global_occupancy_grid->info.width + x];

                // Convert the occupancy grid values to the required image format
                if (occupancy_value == -1)
                    map.at<uchar>(y, x) = 127;  // Unknown space, use gray
                else if (occupancy_value <= 50)
                    map.at<uchar>(y, x) = 255;  // Free space
                else
                    map.at<uchar>(y, x) = 0;    // Obstacle
            }
        }
        ROS_INFO("getting image");
        // Convert the map to sensor_msgs::Image
        cv_bridge::CvImage cv_image;
        cv_image.encoding = "mono8";
        cv_image.image = map;

        sensor_msgs::Image labeling;
        cv_image.toImageMsg(labeling);
        global_labeling = labeling;
        isDataReady = true;
    }
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {   ROS_INFO("getting odom");
        // Assuming the odom message contains pose information
        current_robot_pose.x = odomMsg->pose.pose.position.x;
        current_robot_pose.y = odomMsg->pose.pose.position.y;
        // Assuming the orientation in the quaternion is represented as (x, y, z, w)
        current_robot_pose.theta = tf::getYaw(odomMsg->pose.pose.orientation);
    }

    bool autoPathPlanningCallback(std_srvs::Trigger::Request& req,
                                std_srvs::Trigger::Response& res)
    {  
        // actionlib::SimpleActionClient<ipa_building_msgs::RoomExplorationAction> ac("room_exploration_server", true);
        // Subscribe to the slamware_ros_sdk_server_node.map topic
        sub_map = nh.subscribe("/slamware_ros_sdk_server_node/map", 1, &RoomExplorationClient::mapCallback,this);
        sub_odom = nh.subscribe("/slamware_ros_sdk_server_node/odom", 1,  &RoomExplorationClient::odomCallback, this);

        // read params
        bool use_test_maps;
        priv_nh.param("use_test_maps", use_test_maps, true);
        double resolution;
        priv_nh.param("resolution", resolution, 0.05);
        std::vector<double> origin (3,0);
        priv_nh.param("origin", origin, origin);
        double robot_radius;
        priv_nh.param("robot_radius", robot_radius, 0.2);
        double coverage_radius;
        priv_nh.param("coverage_radius", coverage_radius, 0.2);
        
        // Create a CvImage to hold the received map
        cv_bridge::CvImagePtr cv_image_ptr;
        ROS_INFO("i AM HERE");
        // Example: Accessing the map data (assuming global_labeling is a global variable)
        if (!global_labeling.data.empty())
        {
        cv_image_ptr = cv_bridge::toCvCopy(global_labeling, "mono8");
        cv::Mat map_image = cv_image_ptr->image;

            // Now you can use 'map_image' as the input_map for the room exploration goal
        }
        

        while(!isDataReady)
        {   
            ros::spinOnce();
        }
        isDataReady = false;
        DynamicReconfigureClient drc_exp(nh, "room_exploration_server/set_parameters", "room_exploration_server/parameter_updates");
        // drc_exp.setConfig("path_eps", 25.0);
        // drc_exp.setConfig("map_correction_closing_neighborhood_size", 7);
        std::vector<geometry_msgs::Point32> fov_points(4);

        int planning_mode = 1;
        geometry_msgs::Point32 fov_origin;
        fov_origin.x = 0.;
        fov_origin.y = 0.;
        ipa_building_msgs::RoomExplorationGoal goal;
        goal.input_map = global_labeling;
        goal.map_resolution =global_occupancy_grid->info.resolution;
        goal.map_origin = global_occupancy_grid->info.origin;
        goal.robot_radius = robot_radius; // turtlebot, used for sim 0.177, 0.4
        goal.coverage_radius = coverage_radius;
        goal.field_of_view = fov_points;
        goal.field_of_view_origin = fov_origin;
        goal.starting_position = current_robot_pose;
        goal.planning_mode = planning_mode;
        // shutdown the subscriber (this should be unnecessary)
        sub_map.shutdown();
        sub_odom.shutdown();
        ac.sendGoal(goal);

        

        // Populate the response if needed
        res.success = true;
        res.message = "Auto path planning completed successfully";

        return true;

    }

};


int main(int argc, char** argv) {
    ros::init(argc, argv, "room_exploration_client");
    RoomExplorationClient client;
    ros::spin(); // Keep the node alive
    return 0;
}
