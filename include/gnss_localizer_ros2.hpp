#pragma once

#include <iostream>
#include <memory>
#include <boost/make_shared.hpp>
#include <boost/bind/bind.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "geo_pos_conv.hpp"

#define initial_interval 100 //[ms]
#define cmd_timeout 500      //[ms]

class gnss_localizer_ros2 : public rclcpp::Node
{
public:
  gnss_localizer_ros2(int _plane)
      : rclcpp::Node("gnss_localizser"),plane(_plane),_orientation_ready(false)
  {

    sub_navSatFix = this->create_subscription<sensor_msgs::msg::NavSatFix>("PoSLV/navfix",10,
        [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg){this->navSatFix_callback(msg);});
    pub_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>("global_pose",10);
    pub_gnss_stat = this->create_publisher<std_msgs::msg::Bool>("gnss_stat",10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  };
  ~gnss_localizer_ros2()
  {
  }

private:
  void navSatFix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr &msg)
  {
    rclcpp::Time now = this->get_clock()->now();

    geo_pos_conv geo;
    geo.set_plane(plane);
    geo.llh_to_xyz(msg->latitude, msg->longitude, msg->altitude);


    tf2::Quaternion pose_q;

    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.header.stamp = now;
    pose.header.frame_id = "map";
    pose.pose.position.x = geo.y();
    pose.pose.position.y = geo.x();
    pose.pose.position.z = geo.z();

    // set gnss_stat
    if (pose.pose.position.x == 0.0 || pose.pose.position.y == 0.0 || pose.pose.position.z == 0.0)
    {
      gnss_stat.data = false;
    }
    else
    {
      gnss_stat.data = true;
    }

    double distance = sqrt(pow(pose.pose.position.y - _prev_pose.pose.position.y, 2) +
                          pow(pose.pose.position.x - _prev_pose.pose.position.x, 2));

    if (distance > 0.2)
    {
      yaw = atan2(pose.pose.position.y - _prev_pose.pose.position.y, pose.pose.position.x - _prev_pose.pose.position.x);
      tf2::Quaternion q_temp;
      q_temp.setRPY(0,0,yaw);
      _quat = tf2::toMsg(q_temp);
      _prev_pose = pose;
      _orientation_ready = true;
    }

    if (_orientation_ready)
    {
      pose.pose.orientation = _quat;
      pub_pose->publish(pose);
      pub_gnss_stat->publish(gnss_stat);

      // static tf::TransformBroadcaster br;
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      transformStamped.header.stamp = now;
      transformStamped.header.frame_id = "map";
      transformStamped.child_frame_id = "gnss";
      transformStamped.transform.translation.x = pose.pose.position.x;
      transformStamped.transform.translation.y = pose.pose.position.y;
      transformStamped.transform.translation.z = pose.pose.position.z;
      transformStamped.transform.rotation.w = q.w();
      transformStamped.transform.rotation.x = q.x();
      transformStamped.transform.rotation.y = q.y();
      transformStamped.transform.rotation.z = q.z();

      tf_broadcaster_->sendTransform(transformStamped);
    }
  }


  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_navSatFix;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_gnss_stat;

  sensor_msgs::msg::NavSatFix fix;
  geometry_msgs::msg::PoseStamped _prev_pose;
  geometry_msgs::msg::Quaternion _quat;
  geometry_msgs::msg::TransformStamped transformStamped;
  std_msgs::msg::Bool gnss_stat;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  int plane;
  double yaw;
  bool _orientation_ready;
};
