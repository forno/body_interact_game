#include "body_interact_game/pose_generator.hpp"

namespace
{

std::bernoulli_distribution half_dist {0.5};

}

pose_generator::pose_generator(const pose_generator_config& conf)
  : stand_dist_ {conf.stand_probability},
    both_leg_dist_ {conf.both_leg_average, conf.both_leg_dispersion},
    body_dist_ {conf.body_average, conf.body_dispersion},
    one_side_dist_ {conf.one_side_average, conf.one_side_dispersion}
{
}

void pose_generator::update()
{
  if (stand_dist_(engine_)) { // stand pose
    const auto legs_angle {both_leg_dist_(engine_)};
    const auto body_angle {body_dist_(engine_)};
    head_ = Eigen::AngleAxisd{body_angle, Eigen::Vector3d::UnitZ()} * Eigen::Vector3d::UnitY();
    right_knee_ = Eigen::AngleAxisd{-legs_angle, Eigen::Vector3d::UnitZ()} * -Eigen::Vector3d::UnitY();
    left_knee_ = Eigen::AngleAxisd{legs_angle, Eigen::Vector3d::UnitZ()} * -Eigen::Vector3d::UnitY();

  } else { // one side leg pose
    const auto oneside_angle {one_side_dist_(engine_)};
    if (half_dist(engine_)) { // left side stand
      left_knee_ = -Eigen::Vector3d::UnitY();
      right_knee_ = Eigen::AngleAxisd{-oneside_angle, Eigen::Vector3d::UnitZ()} * -Eigen::Vector3d::UnitY();
      head_ = Eigen::AngleAxisd{-oneside_angle, Eigen::Vector3d::UnitZ()} * Eigen::Vector3d::UnitY();
    } else { // right side stand
      right_knee_ = -Eigen::Vector3d::UnitY();
      left_knee_ = Eigen::AngleAxisd{oneside_angle, Eigen::Vector3d::UnitZ()} * -Eigen::Vector3d::UnitY();
      head_ = Eigen::AngleAxisd{oneside_angle, Eigen::Vector3d::UnitZ()} * Eigen::Vector3d::UnitY();
    }
  }
}
