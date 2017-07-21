#include <body_interact_game/body_analizer.hpp>

#include <Eigen/SVD>

namespace
{

constexpr auto pi {3.141592653589793};

inline Eigen::AngleAxisd get_yaw_inverse_matrix(Eigen::Affine3d pos) noexcept
{
  const auto ypr {pos.rotation().eulerAngles(2, 0, 1)};
  const auto yaw_angle {ypr(0) < pi / 2 ? ypr(2) : ypr(2) - pi};
  return {-yaw_angle, Eigen::Vector3d::UnitY()};
}

}

body_analizer::body_analizer(std::string root, std::size_t target_number)
  : root_ {root},
    target_number_ {std::to_string(target_number)},
    base_name_ {"torso_" + target_number_},
    right_hand_name_ {"right_hand_" + target_number_},
    left_hand_name_ {"left_hand_" + target_number_},
    right_knee_name_ {"right_knee_" + target_number_},
    left_knee_name_ {"left_knee_" + target_number_}
{
}

void body_analizer::update()
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
