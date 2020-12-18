// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: David Vargas Frutos <david.vargas@urjc.es>

#ifndef MARKER_VIZ__MARKER_VIZ_NODE_HPP_
#define MARKER_VIZ__MARKER_VIZ_NODE_HPP_

#include <chrono>
#include <memory>
#include <map>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "mocap_msgs/msg/marker.hpp"
#include "mocap_msgs/msg/markers.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "marker_viz_srvs/srv/set_marker_color.hpp"
#include "marker_viz_srvs/srv/reset_marker_color.hpp"

typedef visualization_msgs::msg::Marker VizMarker;
typedef mocap_msgs::msg::Marker MocapMarker;
typedef mocap_msgs::msg::Markers MocapMarkers;
typedef mocap_msgs::msg::Markers::SharedPtr MocapMarkersSharedPtr;

typedef marker_viz_srvs::srv::SetMarkerColor SetMarkerColor;
typedef marker_viz_srvs::srv::ResetMarkerColor ResetMarkerColor;
typedef std::shared_ptr<marker_viz_srvs::srv::SetMarkerColor::Request> SetRequest;
typedef std::shared_ptr<marker_viz_srvs::srv::SetMarkerColor::Response> SetResponse;
typedef std::shared_ptr<marker_viz_srvs::srv::ResetMarkerColor::Request> ResetRequest;
typedef std::shared_ptr<marker_viz_srvs::srv::ResetMarkerColor::Response> ResetResponse;

class MarkerVisualizer : public rclcpp::Node
{
public:
  MarkerVisualizer();

private:
  void marker_callback(const MocapMarkersSharedPtr msg) const;
  void process_marker(int index, const geometry_msgs::msg::Point & translation) const;

  rclcpp::Publisher<VizMarker>::SharedPtr publisher_;
  rclcpp::Subscription<MocapMarkers>::SharedPtr markers_subscription_;
  geometry_msgs::msg::Vector3 marker_scale_;
  float marker_lifetime_;
  std::string marker_frame_;
  std::string namespace_;
  std_msgs::msg::ColorRGBA default_marker_color_;
  std::map<int, std_msgs::msg::ColorRGBA> marker_color_;
};

#endif  // MARKER_VIZ__MARKER_VIZ_NODE_HPP_
