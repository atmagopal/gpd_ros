#include <gpd_ros/grasp_detection_server.h>



GraspDetectionServer::GraspDetectionServer(ros::NodeHandle& node)
{
	cloud_camera_ = NULL;

	// set camera viewpoint to default origin
	std::vector<double> camera_position;
	node.getParam("camera_position", camera_position);
	view_point_ << camera_position[0], camera_position[1], camera_position[2];

	std::string cfg_file;
	node.param("config_file", cfg_file, std::string(""));
	grasp_detector_ = new gpd::GraspDetector(cfg_file);

	std::string rviz_topic;
	node.param("rviz_topic", rviz_topic, std::string(""));

	// Frames
	base_frame_ = "base";
	grasp_detection_frame_ = "camera_optical_depth_frame";

	// Approach direction map
	std::vector<double>top		{0.0, 0.0,-1.0};
	std::vector<double>front	{1.0, 0.0, 0.0};
	std::vector<double>side_l	{0.0,-1.0, 0.0};
	std::vector<double>side_r	{0.0, 1.0, 0.0};
	direction_map_.insert(std::pair<std::string, std::vector<double>>("top", top ));
	direction_map_.insert(std::pair<std::string, std::vector<double>>("front", front ));
	direction_map_.insert(std::pair<std::string, std::vector<double>>("side_l", side_l ));
	direction_map_.insert(std::pair<std::string, std::vector<double>>("side_r", side_r ));

	workspace_points_.resize(8);

	if (!rviz_topic.empty())
	{
		rviz_plotter_ = new GraspPlotter(node, grasp_detector_->getHandSearchParameters().hand_geometry_);
		use_rviz_ = true;
	}
	else
	{
		use_rviz_ = false;
	}

	// Advertise ROS topic for detected grasps.
	grasps_pub_ 		= node.advertise<gpd_ros::GraspConfigList>("clustered_grasps", 10);
	workspace_pub_  = node.advertise<visualization_msgs::Marker>("visualize_workspace", 1);

	node.getParam("workspace", workspace_);
}

bool GraspDetectionServer::setGPDParams(gpd_ros::detect_params::Request& req, gpd_ros::detect_params::Response& res)
{
	/** Workspace **/
	if(req.three_workspace_points.size() == 3)
	{
		ROS_WARN("3 points needed to define the workspace.");
		res.status.data = false;
		return res.status.data;
	}
	// Assign the 8 vertices that make up the workspace
	workspace_points_.clear();
	setWorkspaceVertices_(workspace_points_, req.three_workspace_points);

	// Publish frame that is aligned to the set workspace
	publishTransform_base2workspace_base(workspace_points_);

	// Draw the workspace box set by request as red lines
	visualizeWorkspace(workspace_points_, eMarkerID::BASE_USER);

	// Assign param struct with workspace min max
	setWorkspace_MinMaxXYZ(myParam_.workspace, workspace_points_);

	// Show gpd min max workspace in WORKSPACE_TABLE_FRAME frame
	std::vector<geometry_msgs::Point> dummy;
	visualizeWorkspace(dummy, eMarkerID::WORKSPACE_TABLE_GPD);


	/** Approach direction **/
	transformApproachDirection_base2camera(myParam_.approach_direction, direction_map_[req.approach_direction.data]);


	/** Direction Angle Threshold **/
	myParam_.thresh_rad = (req.tolerance_direction.data * M_PI) / 180.0;


	/** Base Frame ID **/
	base_frame_ = req.frame_id.data;


	/** Transform Camera2Base **/
	getEigenTransform_(myParam_.transform_camera2base, grasp_detection_frame_, WORKSPACE_TABLE_FRAME);

	res.status.data = true;
	return res.status.data;
}

bool GraspDetectionServer::detectGrasps(gpd_ros::detect_grasps::Request& req, gpd_ros::detect_grasps::Response& res)
{
	ROS_INFO("Received service request ...");

	// 1. Initialize cloud camera.
	cloud_camera_ = NULL;
	const gpd_ros::CloudSources& cloud_sources = req.cloud_indexed.cloud_sources;

	// Set view points.
	Eigen::Matrix3Xd view_points(3, cloud_sources.view_points.size());
	for (int i = 0; i < cloud_sources.view_points.size(); i++)
	{
		view_points.col(i) << cloud_sources.view_points[i].x, cloud_sources.view_points[i].y,
			cloud_sources.view_points[i].z;
	}

	// Set point cloud.
	if (cloud_sources.cloud.fields.size() == 6 && cloud_sources.cloud.fields[3].name == "normal_x"
		&& cloud_sources.cloud.fields[4].name == "normal_y" && cloud_sources.cloud.fields[5].name == "normal_z")
	{
		PointCloudPointNormal::Ptr cloud(new PointCloudPointNormal);
		pcl::fromROSMsg(cloud_sources.cloud, *cloud);

		// TODO: multiple cameras can see the same point
		Eigen::MatrixXi camera_source = Eigen::MatrixXi::Zero(view_points.cols(), cloud->size());
		for (int i = 0; i < cloud_sources.camera_source.size(); i++)
		{
			camera_source(cloud_sources.camera_source[i].data, i) = 1;
		}

		cloud_camera_ = new gpd::util::Cloud(cloud, camera_source, view_points);
	}
	else
	{
		PointCloudRGBA::Ptr cloud(new PointCloudRGBA);
		pcl::fromROSMsg(cloud_sources.cloud, *cloud);

		// TODO: multiple cameras can see the same point
		Eigen::MatrixXi camera_source = Eigen::MatrixXi::Zero(view_points.cols(), cloud->size());
		for (int i = 0; i < cloud_sources.camera_source.size(); i++)
		{
			camera_source(cloud_sources.camera_source[i].data, i) = 1;
		}

		cloud_camera_ = new gpd::util::Cloud(cloud, camera_source, view_points);
		std::cout << "view_points:\n" << view_points << "\n";
	}

	grasp_detector_->preprocessPointCloud(*cloud_camera_, myParam_.workspace, myParam_.transform_camera2base);
	std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps = grasp_detector_->detectGrasps(*cloud_camera_, myParam_);


/* 	// Set the indices at which to sample grasp candidates.
	std::vector<int> indices(req.cloud_indexed.indices.size());
	for (int i=0; i < indices.size(); i++)
	{
		indices[i] = req.cloud_indexed.indices[i].data;
	}
	cloud_camera_->setSampleIndices(indices);

	frame_ = req.cloud_indexed.cloud_sources.cloud.header.frame_id;

	ROS_INFO_STREAM("Received cloud with " << cloud_camera_->getCloudProcessed()->size() << " points, and "
		<< req.cloud_indexed.indices.size() << " samples");

	// 2. Preprocess the point cloud.
	grasp_detector_->preprocessPointCloud(*cloud_camera_);

	// 3. Detect grasps in the point cloud.
	std::vector<std::unique_ptr<gpd::candidate::Hand>> grasps = grasp_detector_->detectGrasps(*cloud_camera_); */

	if (grasps.size() > 0)
	{
		// Visualize the detected grasps in rviz.
		if (use_rviz_)
		{
			rviz_plotter_->drawGrasps(grasps, frame_);
		}

		// Publish the detected grasps.
		gpd_ros::GraspConfigList selected_grasps_msg = GraspMessages::createGraspListMsg(grasps, cloud_camera_header_);
		res.grasp_configs = selected_grasps_msg;
		ROS_INFO_STREAM("Detected " << selected_grasps_msg.grasps.size() << " highest-scoring grasps.");
		return true;
	}

	ROS_WARN("No grasps detected!");
	return false;
}

void GraspDetectionServer::publishTransform_base2workspace_base(const std::vector<geometry_msgs::Point>& workspace_vertices)
{
	auto createUnitVector = [](geometry_msgs::Point p1, geometry_msgs::Point p2)
	{
		Eigen::Vector3d vect_p1_p2(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
		return vect_p1_p2.normalized();
	};
   
	Eigen::Vector3d vectX_workspace;
	Eigen::Vector3d vectY_workspace;
	Eigen::Vector3d vectZ_workspace;

	vectX_workspace = createUnitVector(workspace_vertices[0], workspace_vertices[1]);
	vectY_workspace = createUnitVector(workspace_vertices[1], workspace_vertices[5]);
	vectZ_workspace = createUnitVector(workspace_vertices[0], workspace_vertices[2]);

	// get rotation values of the woskpace unit vectors
	tf::Matrix3x3 rotationMat(  vectX_workspace[0], vectY_workspace[0], vectZ_workspace[0], 
								vectX_workspace[1], vectY_workspace[1], vectZ_workspace[1], 
								vectX_workspace[2], vectY_workspace[2], vectZ_workspace[2]);

	// convert rotation matrix to quaternion
	tf::Quaternion q_tf;
	rotationMat.getRotation(q_tf);
	q_tf = q_tf.normalized();

	// convert
	geometry_msgs::Quaternion q;
	tf::quaternionTFToMsg(q_tf, q);

	// Create and publish transform
	geometry_msgs::TransformStamped static_transformStamped;
	static_transformStamped.header.stamp            = ros::Time::now();
	static_transformStamped.header.frame_id         = base_frame_;
	static_transformStamped.child_frame_id          = WORKSPACE_TABLE_FRAME;
	static_transformStamped.transform.translation.x = 0;
	static_transformStamped.transform.translation.y = 0;
	static_transformStamped.transform.translation.z = 0;
	static_transformStamped.transform.rotation      = q;
	
	static_workspacetf_broadcaster_.sendTransform(static_transformStamped); 
}

void GraspDetectionServer::visualizeWorkspace(const std::vector<geometry_msgs::Point>& workspace, const eMarkerID marker_space)
{
	// Show workspace cube in RViz
	visualization_msgs::Marker ws_cube;
	
	// Initialize marker lines  
	ws_cube.header.stamp         = ros::Time::now();
	ws_cube.ns                   = "grasp_perception";
	ws_cube.action               = visualization_msgs::Marker::ADD;

	if(marker_space == eMarkerID::BASE_USER)
	{
		ws_cube.header.frame_id      = base_frame_;
		ws_cube.points.clear();
		ws_cube.pose.orientation.w   = 1.0;
		ws_cube.id                   = eMarkerID::BASE_USER;
		ws_cube.type                 = visualization_msgs::Marker::LINE_STRIP;
		ws_cube.scale.x              = 0.02;
		ws_cube.color.r              = 1.0;
		ws_cube.color.a              = 0.6;
		ws_cube.lifetime             = ros::Duration();
		ws_cube.text                 = "Workspace from vertices in base_frame_";

		// the order of points is only needed to draw the lines pretty and clean in Rviz
		ws_cube.points.push_back(workspace[0]);
		ws_cube.points.push_back(workspace[1]);
		ws_cube.points.push_back(workspace[3]);
		ws_cube.points.push_back(workspace[2]);
		ws_cube.points.push_back(workspace[0]);
		ws_cube.points.push_back(workspace[4]);
		ws_cube.points.push_back(workspace[5]);
		ws_cube.points.push_back(workspace[1]);
		ws_cube.points.push_back(workspace[3]);
		ws_cube.points.push_back(workspace[7]);
		ws_cube.points.push_back(workspace[6]);
		ws_cube.points.push_back(workspace[2]);
		ws_cube.points.push_back(workspace[0]);
		ws_cube.points.push_back(workspace[4]);
		ws_cube.points.push_back(workspace[6]);
		ws_cube.points.push_back(workspace[7]);
		ws_cube.points.push_back(workspace[5]);

	} else if(marker_space == eMarkerID::WORKSPACE_TABLE_GPD)
	{
		std::vector<double> points = myParam_.workspace;
		
		ws_cube.header.frame_id     = WORKSPACE_TABLE_FRAME;
		ws_cube.id                  = eMarkerID::WORKSPACE_TABLE_GPD;
		ws_cube.type                = visualization_msgs::Marker::CUBE;
		ws_cube.pose.orientation.w  = 1.0;
		// position is midpoint of cube
		ws_cube.pose.position.x     = (points[0] + points[1]) * 0.5;
		ws_cube.pose.position.y     = (points[2] + points[3]) * 0.5;
		ws_cube.pose.position.z     = (points[4] + points[5]) * 0.5;
		// scale is length, width, height
		ws_cube.scale.x             = points[1] - points[0];
		ws_cube.scale.y             = points[3] - points[2];
		ws_cube.scale.z             = points[5] - points[4];
		ws_cube.color.g             = 1.0;
		ws_cube.color.a             = 0.2;
		ws_cube.lifetime            = ros::Duration();
		ws_cube.text                = "Workspace min max in base_frame_";
	} 

	workspace_pub_.publish(ws_cube);     
}

void GraspDetectionServer::setWorkspace_MinMaxXYZ(std::vector<double>& gpd_workspace, const std::vector<geometry_msgs::Point>& workspace_base_frame)
{
	std::vector<geometry_msgs::Point> workspace_table_frame(8);
	std::vector<double> x, y, z;
	x.resize(8);
	y.resize(8);
	z.resize(8);
	
	// publish transform that points to the table
	publishTransform_base2workspace_base(workspace_base_frame);

	// transform ws from base_frame_ frame to WORKSPACE_TABLE_FRAME
	for(short i = 0; i < 8; ++i)
	{
		tf2::doTransform(workspace_base_frame[i],  workspace_table_frame[i], 
			getTransform_(base_frame_, WORKSPACE_TABLE_FRAME));
		x[i] = workspace_table_frame[i].x;
		y[i] = workspace_table_frame[i].y;
		z[i] = workspace_table_frame[i].z;
	}

	// get min max of vertices in WORKSPACE_TABLE_FRAME frame to set workspace
	gpd_workspace.clear();
	gpd_workspace.resize(6);
	gpd_workspace[0] = *std::min_element(x.begin(), x.end());
	gpd_workspace[1] = *std::max_element(x.begin(), x.end());
	
	gpd_workspace[2] = *std::min_element(y.begin(), y.end());
	gpd_workspace[3] = *std::max_element(y.begin(), y.end());
	
	gpd_workspace[4] = *std::min_element(z.begin(), z.end());
	gpd_workspace[5] = *std::max_element(z.begin(), z.end());

}

void GraspDetectionServer::transformApproachDirection_base2camera(Eigen::Vector3d& gpd_approach_direction, const std::vector<double>& approach_direction)
{
	// transform approach_direction and workspace to gpd frame
	geometry_msgs::Vector3 source, target;
	source.x = approach_direction[0];
	source.y = approach_direction[1];
	source.z = approach_direction[2];    
	tf2::doTransform(source,  target, 
		getTransform_(WORKSPACE_TABLE_FRAME, grasp_detection_frame_));

	Eigen::Vector3d temp(target.x, target.y, target.z);
	gpd_approach_direction = temp;
}

void setWorkspaceVertices_(std::vector<geometry_msgs::Point>& workspace_vertices_out, const std::vector<geometry_msgs::Point>& workspace_seed)
{
    double height = 1.0;
    Eigen::Vector3d height_vect;

    workspace_vertices_out.clear();
    workspace_vertices_out.resize(8);

    workspace_vertices_out[0] = workspace_seed[0];
    
		workspace_vertices_out[1] = workspace_seed[1];

    workspace_vertices_out[5] = workspace_seed[2];

    getFourthPoint_HeightVector_(height, workspace_vertices_out[0], workspace_vertices_out[1], workspace_vertices_out[5], 
                                workspace_vertices_out[4], height_vect);

    workspace_vertices_out[2].x = workspace_vertices_out[0].x + height_vect[0];
    workspace_vertices_out[2].y = workspace_vertices_out[0].y + height_vect[1];
    workspace_vertices_out[2].z = workspace_vertices_out[0].z + height_vect[2];

    workspace_vertices_out[3].x = workspace_vertices_out[1].x + height_vect[0];
    workspace_vertices_out[3].y = workspace_vertices_out[1].y + height_vect[1];
    workspace_vertices_out[3].z = workspace_vertices_out[1].z + height_vect[2];

    workspace_vertices_out[6].x = workspace_vertices_out[4].x + height_vect[0];
    workspace_vertices_out[6].y = workspace_vertices_out[4].y + height_vect[1];
    workspace_vertices_out[6].z = workspace_vertices_out[4].z + height_vect[2];

    workspace_vertices_out[7].x = workspace_vertices_out[5].x + height_vect[0];
    workspace_vertices_out[7].y = workspace_vertices_out[5].y + height_vect[1];
    workspace_vertices_out[7].z = workspace_vertices_out[5].z + height_vect[2];

}

void getFourthPoint_HeightVector_(const double height, const geometry_msgs::Point& point1, const geometry_msgs::Point& point2, 
                                 geometry_msgs::Point& point3, geometry_msgs::Point& out_point4, Eigen::Vector3d& out_height_vector) 
{
    // lambda function to get distance between points
    auto getDistance = [](geometry_msgs::Point p1, geometry_msgs::Point p2)
    {
        return sqrt(pow((p1.x - p2.x), 2) +
                    pow((p1.y - p2.y), 2) +
                    pow((p1.z - p2.z), 2));
    };

    // lambda function to get fourth point that completes bottom rectangle, given three of the points
    auto getFourthRectanglePoint = [](geometry_msgs::Point p1, geometry_msgs::Point p2, geometry_msgs::Point pWidth)
    { 
        // midpoint of p1 to p3 diagonal, rel. to p1
        double x_m  = p1.x + (pWidth.x - p1.x)/2;
        double y_m  = p1.y + (pWidth.y - p1.y)/2;
        double z_m  = p1.z + (pWidth.z - p1.z)/2;

        geometry_msgs::Point fourthPoint;

        // fourthPoint = (midpoint of p1 to p3 diagonal, rel. to p1) + (vector of p2 to diagonal midpoint)
        fourthPoint.x = x_m + (x_m - p2.x);
        fourthPoint.y = y_m + (y_m - p2.y);
        fourthPoint.z = z_m + (z_m - p2.z);

        return fourthPoint;
    };

    // lambda function to get normal vector (pointing up from ground) given three points
    auto getScaledNormalVector = [](geometry_msgs::Point pMid, geometry_msgs::Point p1, geometry_msgs::Point p2, double scale)
    {      
        /** 
         * normal vector = cross product of two vector (3 points)
         * 
         * right_hand rule: vectA - forefinger; vectB - middle finger; resultant: vectC - thumb
         * vectC = vectA x vectB :: x is cross product
         * 
         * vectA - vector pointMiddle->point1; vectB - vector pointMiddle->point2 
         */
        Eigen::Vector3d vectA(p1.x - pMid.x, p1.y - pMid.y, p1.z - pMid.z);
        Eigen::Vector3d vectB(p2.x - pMid.x, p2.y - pMid.y, p2.z - pMid.z);
        Eigen::Vector3d vectC = vectA.cross(vectB);

        // make unit vector of normal
        vectC = vectC.normalized();

        // scale unit vector to given length/depth
        Eigen::UniformScaling<double> tf_scale(scale);
        Eigen::Vector3d scaledNormalVector = tf_scale * vectC;

        return scaledNormalVector;
    };

    double distance_p1_p3 = getDistance(point1, point3);
    double distance_p2_p3 = getDistance(point2, point3);

    // Compare distance of point3 to point1 and point2
    // if closer to point1 (left): assign point3 as point4, calculate point3
    // else: calculate point4
    if (distance_p1_p3 < distance_p2_p3)
    { 
        ROS_WARN("The third point given is closer to the left point, assigned as left rear point now.");
        out_point4 = point3; 
        point3 = getFourthRectanglePoint(point2, point1, out_point4); // point2---point4 makes the diagonal
        out_height_vector = getScaledNormalVector(point1, point2, out_point4, height);
    } else 
    {
        out_point4 = getFourthRectanglePoint(point1, point2, point3); // point1---point3 makes the diagonal
        out_height_vector = getScaledNormalVector(point2, point3, point1, height);
    } 
}

geometry_msgs::TransformStamped getTransform_(std::string source_frame, std::string target_frame)
{
	tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
	geometry_msgs::TransformStamped source2target;
	ros::Duration timeout(1.0);

	try {
		source2target = tfBuffer.lookupTransform(target_frame, source_frame,
				ros::Time(0), timeout);
	} catch (tf2::TransformException &ex) {
		ROS_WARN("%s", ex.what());
		ROS_INFO("Trying again with longer (2s) timeout.");
		source2target = tfBuffer.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(2.0) );
	}

	return source2target;
}

void getEigenTransform_(Eigen::Affine3d& out_transform, std::string source_frame, std::string target_frame)
{
	geometry_msgs::TransformStamped source2target;
	source2target = getTransform_(source_frame, target_frame);
	tf::transformMsgToEigen(source2target.transform, out_transform);
}

int main(int argc, char** argv)
{
	// seed the random number generator
	std::srand(std::time(0));

	// initialize ROS
	ros::init(argc, argv, "detect_grasps_server");
	ros::NodeHandle node("~");

	GraspDetectionServer grasp_detection_server(node);

	ros::ServiceServer service = node.advertiseService("detect_grasps", &GraspDetectionServer::detectGrasps,
																										 &grasp_detection_server);
																										 
	ros::ServiceServer service2 = node.advertiseService("setGPDParams", &GraspDetectionServer::setGPDParams,
																										 &grasp_detection_server);
	
	ROS_INFO("Grasp detection service is waiting for a point cloud ...");

	ros::spin();

	return 0;
}
