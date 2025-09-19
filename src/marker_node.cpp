/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker.hpp>


using namespace std::chrono_literals;
  
class UPOMarker : public rclcpp::Node
{
  public:
    UPOMarker()
    : Node("upo_marker"), count_(0)
    {
      publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("upo_marker", 10);
      timer_ = this->create_wall_timer(
      100ms, std::bind(&UPOMarker::timer_callback, this));

      init_parameters();
      init_marker();
      
    }

  private:
    void timer_callback()
    {
      marker.header.stamp = this->get_clock()->now();
      publisher_->publish(marker);
    }

    void init_parameters() {
      this->declare_parameter("frame_id", "base_link");
      this->declare_parameter("model", "m600");
      this->declare_parameter("scale", 1.0f);
      this->declare_parameter("position_x", 0.0f);
      this->declare_parameter("position_y", 0.0f);
      this->declare_parameter("position_z", 0.0f);
      this->declare_parameter("radius", 2.0f);
      this->declare_parameter("points", 50);
      this->declare_parameter("radius", 1.0f);
    }

    void init_marker() {
      marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;

      // Set the frame ID.
      std::string base_link = this->get_parameter("frame_id").as_string();

      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      marker.pose.position.x = 0;
      marker.pose.position.y = 0;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0;
      marker.pose.orientation.y = 0;
      marker.pose.orientation.z = 0;
      marker.pose.orientation.w = 1;

      std::string model_name;
      model_name = this->get_parameter("model").as_string();
      marker.mesh_resource = "package://upo_markers/Resource/" + model_name + ".dae";

      if (model_name == "raposa") {
        marker.pose.position.x = -0.3;
        marker.pose.position.y = -0.22;
        marker.pose.position.z = -0.15;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0.70711;
        marker.pose.orientation.w = 0.70711;
      }

      if (model_name == "m600" || model_name == "m100") {
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.70711;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 0.70711;
      }
      marker.id = 0;

      // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
      marker.action = visualization_msgs::msg::Marker::ADD;

      // Set the scale of the marker
      
      marker.scale.y = marker.scale.z = marker.scale.x = this->get_parameter("scale").as_double(); //TODO scale independent (x,y,z)

      marker.pose.position.x = this->get_parameter("position_x").as_double();
      marker.pose.position.y = this->get_parameter("position_y").as_double();
      marker.pose.position.z = this->get_parameter("position_z").as_double();
      
      marker.color.r = this->get_parameter("r").as_double();
      marker.color.g = this->get_parameter("g").as_double();
      marker.color.b = this->get_parameter("b").as_double();
      
      marker.color.a = this->get_parameter("alpha").as_double();

      // Circle marker. Could  be used for representing regular polygons
      if (model_name == "circle" || model_name == "polygon") {
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;
        marker.scale.y = marker.scale.z = 0.0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        // Define the line
        double radius = this->get_parameter("radius").as_double();
        geometry_msgs::msg::Point p;
        int n_points = this->get_parameter("points").as_int();
        double inc = 2.0 * M_PI / static_cast<double>(n_points);
        p.z = 0;
        for (int i = 0; i < n_points; i++) {
          p.x = radius * cos ( static_cast<double>(i) * inc );
          p.y = radius * sin ( static_cast<double>(i) * inc );
          marker.points.push_back(p);
        }
        // Close the circle
        p.x = radius ;
        p.y = 0.0;
        marker.points.push_back(p);
      }

      
    }


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    size_t count_;
    visualization_msgs::msg::Marker marker;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UPOMarker>());
  rclcpp::shutdown();
  return 0;
}
