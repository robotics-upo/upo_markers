#include <ros/ros.h>
#include <ros/node_handle.h>
#include <visualization_msgs/MarkerArray.h>

double g_width, g_height, g_length;

visualization_msgs::MarkerArray buildRobotModel(std::string frame_id)
{
	int id = 0;
	visualization_msgs::MarkerArray model;
	visualization_msgs::Marker marker;
	geometry_msgs::Point p;
	
	// Add base link
	marker.header.frame_id = frame_id;
	marker.header.stamp = ros::Time::now();
	marker.ns = "arco_model";
	marker.id = id++;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = g_height;
	marker.pose.orientation.x = 0;
	marker.pose.orientation.y =  0.0;
	marker.pose.orientation.z =  0.0;
	marker.pose.orientation.w =  1.0;
	marker.scale.x = g_length;
	marker.scale.y = g_width;
	marker.scale.z = g_height;
	marker.color.a = 1.0; 
	marker.color.r = 200.0/255.0;
	marker.color.g = 200.0/255.0;
	marker.color.b = 200.0/255.0;
	marker.points.clear();
	model.markers.push_back(marker);

	// Add front-left wheel
	marker.id = id++;
  marker.type = visualization_msgs::Marker::CUBE;
	marker.pose.position.x = 0.231;
	marker.pose.position.y = 0.31;
	marker.pose.position.z = 0.05;
	marker.pose.orientation.x =  0.707;
	marker.pose.orientation.y =  0.0;
	marker.pose.orientation.z =  0.0;
	marker.pose.orientation.w =  0.707;
	marker.scale.x = 0.2;
	marker.scale.y = 0.2;
	marker.scale.z = 0.1;
  marker.color.r = 50.0/255.0;
  marker.color.g = 50.0/255.0;
  marker.color.b = 50.0/255.0;
	marker.points.clear();
	model.markers.push_back(marker);

	// Add front-right wheel
	marker.id = id++;
  marker.type = visualization_msgs::Marker::CUBE;
	marker.pose.position.x = 0.231;
	marker.pose.position.y = -0.31;
	marker.pose.position.z = 0.05;
	marker.pose.orientation.x =  0.707;
	marker.pose.orientation.y =  0.0;
	marker.pose.orientation.z =  0.0;
	marker.pose.orientation.w =  0.707;
	marker.scale.x = 0.2;
	marker.scale.y = 0.2;
	marker.scale.z = 0.1;
  marker.color.r = 50.0/255.0;
  marker.color.g = 50.0/255.0;
  marker.color.b = 50.0/255.0;
	marker.points.clear();
	model.markers.push_back(marker);

	// Add back-right wheel
	marker.id = id++;
	marker.pose.position.x = -0.231;
	marker.pose.position.y = -0.31;
	marker.pose.position.z = 0.05;
	marker.pose.orientation.x =  0.707;
	marker.pose.orientation.y =  0.0;
	marker.pose.orientation.z =  0.0;
	marker.pose.orientation.w =  0.707;
	marker.scale.x = 0.2;
	marker.scale.y = 0.2;
	marker.scale.z = 0.1;
	marker.points.clear();
	model.markers.push_back(marker);
	
	// Add back-left wheel
	marker.id = id++;
	marker.pose.position.x = -0.231;
	marker.pose.position.y = 0.31;
	marker.pose.position.z = 0.05;
	marker.pose.orientation.x =  0.707;
	marker.pose.orientation.y =  0.0;
	marker.pose.orientation.z =  0.0;
	marker.pose.orientation.w =  0.707;
	marker.scale.x = 0.2;
	marker.scale.y = 0.2;
	marker.scale.z = 0.1;
	marker.points.clear();
	model.markers.push_back(marker);

	// Add structure (TODO)


	return model;
}

int main( int argc, char **argv)
{
	// Setup ROS
	ros::init(argc, argv, "siar_model_viz");
	ros::NodeHandle nh;
	ros::NodeHandle lnh("~");
	ros::Publisher pubModel;
	ros::Subscriber subStatus;
	
	// Read parameters
	std::string frame_id;
	double hz;
	if(!lnh.getParam("frame_id", frame_id))
		frame_id = "arco/base_link";	
	if(!lnh.getParam("hz", hz))
		hz = 10.0;	
	
	// Create the publisher and subscribers
	lnh.param("width", g_width, 0.714);
	lnh.param("length", g_length, 0.723);
  lnh.param("height", g_height, 0.15);
	pubModel = lnh.advertise<visualization_msgs::MarkerArray>("arco_model", 0);
	// subStatus = lnh.subscribe("siar_status", 1, statusCallback);
	
	// Loop for ever
	ros::Rate loop_rate(hz);
	while(ros::ok())
	{
		// Create the model
		visualization_msgs::MarkerArray model = buildRobotModel(frame_id);	
		
		// Pibllish model
		pubModel.publish(model);
		
		// Update ros and sleep
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}


