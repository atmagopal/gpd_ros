/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Andreas ten Pas
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef GRASP_DETECTION_SERVER_H_
#define GRASP_DETECTION_SERVER_H_

#include <math.h>

// ROS
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Eigen and TF
#include <Eigen/Geometry>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// GPD
#include <gpd/util/cloud.h>
#include <gpd/grasp_detector.h>

// this project (services)
#include <gpd_ros/detect_grasps.h>
#include <gpd_ros/detect_params.h>

// this project (messages)
#include <gpd_ros/GraspConfig.h>
#include <gpd_ros/GraspConfigList.h>

// this project (headers)
#include <gpd_ros/grasp_messages.h>
#include <gpd_ros/grasp_plotter.h>

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudPointNormal;

#define WORKSPACE_TABLE_FRAME "workspace_frame"

class GraspDetectionServer
{
public:

	/**
	 * \brief Constructor.
	 * \param node the ROS node
	*/
	GraspDetectionServer(ros::NodeHandle& node);

	/**
	 * \brief Destructor.
	*/
	~GraspDetectionServer()
	{
		delete cloud_camera_;
		delete grasp_detector_;
		delete rviz_plotter_;
	}

	/**
	 * \brief Service callback for detecting grasps.
	 * \param req the service request
	 * \param res the service response
	 */
	bool detectGrasps(gpd_ros::detect_grasps::Request& req, gpd_ros::detect_grasps::Response& res);

	bool setGPDParams(gpd_ros::detect_params::Request& req, gpd_ros::detect_params::Response& res);

private:

	/**
	 * @brief Purely for code readabillity, to switch workspace cube visualization 
	 */
	enum eMarkerID
	{   
		// <frame shorthand>_<workspace set by/for?> = <marker id>
		BASE_USER           = 99,
		WORKSPACE_TABLE_GPD = 98
	};

	
	void setWorkspace_MinMaxXYZ(std::vector<double>& gpd_workspace, const std::vector<geometry_msgs::Point>& workspace_base_frame);

	void publishTransform_base2workspace_base(const std::vector<geometry_msgs::Point>& workspace_vertices);

	void transformApproachDirection_base2camera(Eigen::Vector3d& gpd_approach_direction, const std::vector<double>& approach_direction);

	void visualizeWorkspace(const std::vector<geometry_msgs::Point>& workspace, const eMarkerID marker_space);

	ros::Publisher grasps_pub_;             ///< ROS publisher for grasp list messages
	ros::Publisher workspace_pub_;

	std_msgs::Header cloud_camera_header_;  ///< stores header of the point cloud
	std::string frame_;                     ///< point cloud frame

	gpd::GraspDetector* grasp_detector_;    ///< used to run the grasp pose detection
	gpd::util::Cloud* cloud_camera_;        ///< stores point cloud with (optional) camera information and surface normals
	GraspPlotter* rviz_plotter_;            ///< used to plot detected grasps in rviz

	bool use_rviz_;                         ///< if rviz is used for visualization instead of PCL
	std::vector<double> workspace_;         ///< workspace limits
	Eigen::Vector3d view_point_;             ///< (input) view point of the camera onto the point cloud

	tf2_ros::StaticTransformBroadcaster static_workspacetf_broadcaster_;
	std::string base_frame_;
	std::string grasp_detection_frame_;
	gpd::DetectParams myParam_;
	std::vector<geometry_msgs::Point> workspace_points_;
	std::map<std::string, std::vector<double>> direction_map_;
};

void setWorkspaceVertices_(std::vector<geometry_msgs::Point>& workspace_vertices_out, const std::vector<geometry_msgs::Point>& workspace_seed);

void getFourthPoint_HeightVector_(const double height, const geometry_msgs::Point& point1, const geometry_msgs::Point& point2, 
																 geometry_msgs::Point& point3, geometry_msgs::Point& out_point4, Eigen::Vector3d& out_height_vector); 

geometry_msgs::TransformStamped getTransform_(std::string source_frame, std::string target_frame);

void getEigenTransform_(Eigen::Affine3d& out_transform, std::string source_frame, std::string target_frame);

#endif /* GRASP_DETECTION_SERVER_H_ */
