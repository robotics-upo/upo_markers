#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

class ArcoMarker : public rclcpp::Node
{
  public:
    ArcoMarker()
    : Node("arco_marker")
    {
      publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("arco_marker", 10);
      timer_ = this->create_wall_timer(
      100ms, std::bind(&ArcoMarker::timer_callback, this));
	  message_ = buildRobotModel(frame_id);

	  this->declare_parameter("frame_id", "arco/base_link");
	  frame_id = this->get_parameter("frame_id").as_string();	
    }

  private:
    void timer_callback()
    {
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message_);
    }

	visualization_msgs::msg::MarkerArray buildRobotModel(std::string frame_id, int id = 0)
	{
		visualization_msgs::msg::MarkerArray model;
		visualization_msgs::msg::Marker marker;
		geometry_msgs::msg::Point p;
		
		// Add base link
		marker.header.frame_id = frame_id;
		marker.header.stamp = this->get_clock()->now();
		marker.ns = "arco_model";
		marker.id = id++;
		marker.type = visualization_msgs::msg::Marker::CUBE;
		marker.action = visualization_msgs::msg::Marker::ADD;
		marker.pose.position.x = 0;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;
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
		double wheel_height = -0.05;
		marker.type = visualization_msgs::msg::Marker::CYLINDER;
		marker.action = visualization_msgs::msg::Marker::ADD;
		marker.pose.position.x = 0.231;
		marker.pose.position.y = 0.31;
		marker.pose.position.z = wheel_height;
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
		marker.pose.position.x = 0.231;
		marker.pose.position.y = -0.31;
		model.markers.push_back(marker);

		// Add back-right wheel
		marker.id = id++;
		marker.pose.position.x = -0.231;
		marker.pose.position.y = -0.31;
		model.markers.push_back(marker);
		
		// Add back-left wheel
		marker.id = id++;
		marker.pose.position.x = -0.231;
		marker.pose.position.y = 0.31;
		marker.points.clear();
		model.markers.push_back(marker);

		// Add structure (TODO)
		return model;
	}
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
	visualization_msgs::msg::MarkerArray message_;
	std::string frame_id = "arco/base_link";

	double g_width = 0.714;
	double g_length = 0.723;
	double g_height = 0.15;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArcoMarker>());
  rclcpp::shutdown();
  return 0;
}



