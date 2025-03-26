#pragma once

#include <iostream>
#include <memory>
#include <boost/make_shared.hpp>
#include <boost/bind/bind.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/path.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

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
    sub_ahrs = this->create_subscription<sensor_msgs::msg::Imu>("PoSLV/ahrs",10,
        [this](const sensor_msgs::msg::Imu::SharedPtr msg){this->arhs_callback(msg);});
    pub_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>("global_pose",10);
    pub_path = this->create_publisher<nav_msgs::msg::Path>("global_pose_past",10);
    pub_gnss_stat = this->create_publisher<std_msgs::msg::Bool>("gnss_stat",10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  };

  ~gnss_localizer_ros2()
  {
  }


private:

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr  sub_navSatFix;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr        sub_ahrs;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr             pub_path;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr             pub_gnss_stat;

  std_msgs::msg::Bool                   gnss_stat;
  sensor_msgs::msg::NavSatFix           fix;
  geometry_msgs::msg::PoseStamped       _prev_pose;
  geometry_msgs::msg::Quaternion        _quat;
  geometry_msgs::msg::TransformStamped  ts_gnss;

  nav_msgs::msg::Path  past_path_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  int plane;
  double yaw;
  bool _orientation_ready;

  void navSatFix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr &msg)
  {
    rclcpp::Time time_now = this->get_clock()->now();

    geo_pos_conv geo;
    geo.set_plane(plane);
    geo.llh_to_xyz(msg->latitude, msg->longitude, msg->altitude);

    geometry_msgs::msg::PoseStamped gnss_pose;
    gnss_pose.header        = msg->header;
    gnss_pose.header.stamp  = time_now;
    gnss_pose.header.frame_id = "map";
    gnss_pose.pose.position.x = geo.y();
    gnss_pose.pose.position.y = geo.x();
    // gnss_pose.pose.position.z = geo.z();
    gnss_pose.pose.position.z = 0.0;

    // set gnss_stat
    if (gnss_pose.pose.position.x == 0.0 || gnss_pose.pose.position.y == 0.0 || gnss_pose.pose.position.z == 0.0)
    {
      gnss_stat.data = false;
    }
    else
    {
      gnss_stat.data = true;
    }

    // double distance = sqrt(pow(pose.pose.position.y - _prev_pose.pose.position.y, 2) +
    //                       pow(pose.pose.position.x - _prev_pose.pose.position.x, 2));

    // if (distance > 0.2)
    // {
    //   yaw = atan2(pose.pose.position.y - _prev_pose.pose.position.y, pose.pose.position.x - _prev_pose.pose.position.x);
    //   tf2::Quaternion q_temp;
    //   q_temp.setRPY(0,0,yaw);
    //   _quat = tf2::toMsg(q_temp);
    //   _prev_pose = pose;
    //   _orientation_ready = true;
    // }

    if (_orientation_ready)
    {
      gnss_pose.pose.orientation = _quat;

      ts_gnss.header.stamp      = time_now;
      ts_gnss.header.frame_id   = "map"; //world
      ts_gnss.child_frame_id    = "gnss_temp_link";   //gnss
      ts_gnss.transform.translation.x = gnss_pose.pose.position.x;
      ts_gnss.transform.translation.y = gnss_pose.pose.position.y;
      ts_gnss.transform.translation.z = gnss_pose.pose.position.z;
      ts_gnss.transform.rotation      = _quat;

      // tf_broadcaster_->sendTransform(transformStamped);

      try
      {
        geometry_msgs::msg::PoseStamped       gnss_base_link_pose;
        geometry_msgs::msg::TransformStamped  trans_gnss_to_base_link 
          = tf_buffer_->lookupTransform ( "base_link", "gnss_temp_link", tf2::TimePointZero );
        tf2::doTransform (gnss_pose, gnss_base_link_pose, trans_gnss_to_base_link);
        gnss_base_link_pose.header.stamp = time_now;
        gnss_base_link_pose.header.frame_id = "map";

        // `map -> base_link` のTFメッセージを作成
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp    = time_now;
        transform_stamped.header.frame_id = "map";
        transform_stamped.child_frame_id  = "base_link";

        transform_stamped.transform.translation.x = gnss_base_link_pose.pose.position.x;
        transform_stamped.transform.translation.y = gnss_base_link_pose.pose.position.y;
        transform_stamped.transform.translation.z = gnss_base_link_pose.pose.position.z;

        transform_stamped.transform.rotation = gnss_base_link_pose.pose.orientation;
        pub_pose->publish (gnss_base_link_pose);
        pub_gnss_stat->publish (gnss_stat);

        // `tf2` で `map -> base_link` の変換をブロードキャスト
        tf_broadcaster_->sendTransform(transform_stamped);


        // Path の生成
        past_path_.header.stamp = time_now;
        past_path_.header.frame_id = "map";
        past_path_.poses.push_back (gnss_base_link_pose);
        
        double last_sample_time =  rclcpp::Time(time_now).nanoseconds() / 1e9 - rclcpp::Time (past_path_.poses.front().header.stamp).nanoseconds() / 1e9;

        std::cout << rclcpp::Time (past_path_.poses.front().header.stamp).nanoseconds() / 1e9 << std::endl;
        std::cout <<rclcpp::Time(time_now).nanoseconds() / 1e9 << std::endl;
        std::cout << last_sample_time << std::endl << std::endl;

        if (last_sample_time > 10.0 )
        {
          past_path_.poses.erase(past_path_.poses.begin()); // 先頭の行を削除
        } 
        pub_path->publish(past_path_);

      }
      catch (const tf2::TransformException &ex)
      {
          RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
      }
    }

  }

  void arhs_callback(const sensor_msgs::msg::Imu::SharedPtr &msg)
  {
    _quat = msg->orientation;
    _orientation_ready = true;
  }
};
