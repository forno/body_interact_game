#ifndef BODY_ANALIZER_H
#define BODY_ANALIZER_H

#include <string>

#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>

class body_analizer
{
  std::string root_;
  std::string target_number_;
  std::string base_name_;
  std::string right_hand_name_;
  std::string left_hand_name_;
  std::string right_knee_name_;
  std::string left_knee_name_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_ {tf_buffer_};
  Eigen::Vector3d head_ {};
  Eigen::Vector3d right_hand_ {};
  Eigen::Vector3d left_hand_ {};
  Eigen::Vector3d right_knee_ {};
  Eigen::Vector3d left_knee_ {};

public:
  body_analizer(std::string root, std::size_t target_number);

  void update();

  Eigen::Vector3d get_head() const noexcept
  {
    return head_;
  }

  Eigen::Vector3d get_right_hand() const noexcept
  {
    return right_hand_;
  }

  Eigen::Vector3d get_left_hand() const noexcept
  {
    return left_hand_;
  }

  Eigen::Vector3d get_right_knee() const noexcept
  {
    return right_knee_;
  }

  Eigen::Vector3d get_left_knee() const noexcept
  {
    return left_knee_;
  }

private:
  Eigen::Affine3d get_position(std::string target, ros::Time time) const
  {
    return tf2::transformToEigen(tf_buffer_.lookupTransform(root_, target, time));
  }
};

#endif
