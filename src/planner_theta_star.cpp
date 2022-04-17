// Copyright (c) 2022 Eric Slaghuis
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

/* *********************************************************
 * Plugin to the Navigation Lite Planner Server
 * Planning algorith: Theta Star
 * *********************************************************/

#include <navigation_lite/regular_planner.hpp>
#include "planner_plugins/costmapadaptor.hpp"
#include <cmath>

namespace planner_plugins
{
  class ThetaStar : public navigation_lite::RegularPlanner
  {
    public:
      void configure(const rclcpp::Node::SharedPtr parent, 
                 std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<octomap::OcTree> costmap) override
      {
        parent_node_ = parent;
        name_ = name;
        tf_buffer_ = tf;
        costmap_ = costmap;
    
        weight_ = parent_node_->declare_parameter<float>("weight", 100.0);
      }
      
      nav_msgs::msg::Path createPlan( const geometry_msgs::msg::PoseStamped & start,
                                      const geometry_msgs::msg::PoseStamped & goal) override
      {            
        RCLCPP_DEBUG(parent_node_->get_logger(), "Requested ThetaStar to plan a path from [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f]",
        start.pose.position.x,
        start.pose.position.y,
        start.pose.position.z,
        goal.pose.position.x,
        goal.pose.position.y,
        goal.pose.position.z);
        
        nav_msgs::msg::Path global_path;
        
        // Instantiating our costmap adaptor
        CostmapAdaptor adaptor(costmap_, 3);
        // ... and the path finder
        Pathfinder pathfinder(adaptor, weight_);    //Weight is used to tune search performance
    
        octomap::point3d plan_start(start.pose.position.x, start.pose.position.y, start.pose.position.z);
        octomap::point3d plan_goal(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
    
    
        auto found_path = pathfinder.search( plan_start, plan_goal );
    
        rclcpp::Time now = parent_node_->get_clock()->now();
        global_path.header.stamp = now;
        global_path.header.frame_id = start.header.frame_id;
    
        // If the list only has one member, (typcally the start) no valid path could be found.
        // at least the start and the destination is required to be a valid path.
        for(const auto& waypoint : found_path) {
          geometry_msgs::msg::PoseStamped pose;
          pose.header.stamp = now;
          pose.header.frame_id = start.header.frame_id;
          pose.pose.position.x = (double) waypoint.x();
          pose.pose.position.y = (double) waypoint.y();
          pose.pose.position.z = (double) waypoint.z();      
          global_path.poses.push_back(pose);
    
        }
        
        return global_path;
      }
      
      void initialize(double side_length) override
      {
        side_length_ = side_length;
      }

      double area() override
      {
        return 0.5 * side_length_ * getHeight();
      }

      double getHeight()
      {
        return sqrt((side_length_ * side_length_) - ((side_length_ / 2) * (side_length_ / 2)));
      }

    protected:
      double side_length_;
      double side_length;
      rclcpp::Node::SharedPtr parent_node_;
      std::string name_;
      std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
      std::shared_ptr<octomap::OcTree> costmap_;
  
      float weight_;

  };
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(planner_plugins::ThetaStar, navigation_lite::RegularPlanner)
