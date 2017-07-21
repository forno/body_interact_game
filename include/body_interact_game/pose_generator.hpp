#ifndef BODY_POSE_GENERATOR_H
#define BODY_POSE_GENERATOR_H

#include <random>

#include <Eigen/Geometry>

struct pose_generator_config
{
  double stand_probability;
  double both_leg_average;
  double both_leg_dispersion;
  double body_average;
  double body_dispersion;
  double one_side_average;
  double one_side_dispersion;
};

class pose_generator
{
  std::bernoulli_distribution stand_dist_;
  std::normal_distribution<> both_leg_dist_;
  std::normal_distribution<> body_dist_;
  std::normal_distribution<> one_side_dist_;
  std::default_random_engine engine_ {std::random_device{}()};
  Eigen::Vector3d head_ {};
  Eigen::Vector3d right_hand_ {};
  Eigen::Vector3d left_hand_ {};
  Eigen::Vector3d right_knee_ {};
  Eigen::Vector3d left_knee_ {};

public:
  pose_generator(const pose_generator_config& conf);

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
};

#endif
