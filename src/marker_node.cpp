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

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "upo_marker");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  ros::Rate r(1);  

  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;

  // Set the frame ID.  
  pn.param("base_frame_id", marker.header.frame_id, std::string("/base_link"));

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = -0.4;
  marker.pose.position.y = -0.25;
  marker.pose.position.z = -0.15;
  marker.pose.orientation.x = 0.5;
  marker.pose.orientation.y = 0.5;
  marker.pose.orientation.z = 0.5;
  marker.pose.orientation.w = 0.5;

  std::string default_namespace = "upo_marker";
  std::string model_name;
  pn.param("model", model_name, std::string("raposa"));
  marker.mesh_resource = "package://upo_markers/Resource/" + model_name + ".dae";
  default_namespace = model_name;

  if (model_name == "m600" || model_name == "m100") {
    // TODO: Change the position for m600?
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.5;
    marker.pose.orientation.y = 0.5;
    marker.pose.orientation.z = 0.5;
    marker.pose.orientation.w = 0.5;
  } 
  
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  pn.param("namespace", marker.ns, default_namespace);
  marker.id = 0;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  
  // Set the scale of the marker
  pn.param("scale", marker.scale.x, 1.0);
  marker.scale.y = marker.scale.z = marker.scale.x;

  pn.param("scale_x", marker.scale.x, marker.scale.x);
  pn.param("scale_y", marker.scale.y, marker.scale.y);
  pn.param("scale_z", marker.scale.z, marker.scale.z);

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.9f;
  marker.color.g = 0.9f;
  marker.color.b = 0.9f;
  marker.color.a = 1.0f;

  marker.lifetime = ros::Duration();


 
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>(marker.ns, 1);

  while (ros::ok())
  {
    marker.header.stamp = ros::Time::now();  

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    ROS_INFO_ONCE("Publishing marker");
    marker_pub.publish(marker);

    r.sleep();
  }
}
