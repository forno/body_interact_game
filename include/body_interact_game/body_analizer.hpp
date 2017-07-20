#ifndef BODY_ANALIZER_H
#define BODY_ANALIZER_H

#include <string>

#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>
#include <Eigen/SVD>

constexpr auto pi {3.141592653589793};

inline Eigen::AngleAxisd get_yaw_inverse_matrix(Eigen::Affine3d pos) noexcept
{
  const auto ypr {pos.rotation().eulerAngles(2, 0, 1)};
  const auto yaw_angle {ypr(0) < pi / 2 ? ypr(2) : ypr(2) - pi};
  return {-yaw_angle, Eigen::Vector3d::UnitY()};
}

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
  body_analizer(std::string root, std::size_t target_number)
    : root_ {root},
      target_number_ {std::to_string(target_number)},
      base_name_ {"torso_" + target_number_},
      right_hand_name_ {"right_hand_" + target_number_},
      left_hand_name_ {"left_hand_" + target_number_},
      right_knee_name_ {"right_knee_" + target_number_},
      left_knee_name_ {"left_knee_" + target_number_}
  {
  }

  void update()
  {
    const auto current_time {ros::Time{0}};
    const auto base_pos {get_position(base_name_, current_time)};
    const auto yaw_canceller {get_yaw_inverse_matrix(base_pos)};
    head_ = yaw_canceller * base_pos.rotation() * Eigen::Vector3d::UnitY();
    const auto base_vec {base_pos.translation()};
    right_hand_ = yaw_canceller * (get_position(right_hand_name_, current_time).translation() - base_vec);
    left_hand_ = yaw_canceller * (get_position(left_hand_name_, current_time).translation() - base_vec);
    right_knee_ = yaw_canceller * (get_position(right_knee_name_, current_time).translation() - base_vec);
    left_knee_ = yaw_canceller * (get_position(left_knee_name_, current_time).translation() - base_vec);
  }

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
