#ifndef BODY_INTERACT_GAME_H
#define BODY_INTERACT_GAME_H

#include <rviz_visual_tools/rviz_visual_tools.h>

#include "body_interact_game/body_analizer.hpp"
#include "body_interact_game/pose_generator.hpp"
#include "body_interact_game/scorer.hpp"

#include <Eigen/Geometry>

class interact_game
{
  body_analizer ba_;
  pose_generator pg_;
  scorer s_;
  std::size_t interval_;

public:
  interact_game(const std::string& root, std::size_t number, const pose_generator_config& pgc, std::size_t interval);

  void update();

  template<typename T>
  void render(T visualizer) const
  {
    visualizer(ba_.get_head(), rviz_visual_tools::BLUE);
    visualizer(ba_.get_right_knee(), rviz_visual_tools::BLUE);
    visualizer(ba_.get_left_knee(), rviz_visual_tools::BLUE);
    visualizer(pg_.get_head(), rviz_visual_tools::GREEN);
    visualizer(pg_.get_right_knee(), rviz_visual_tools::GREEN);
    visualizer(pg_.get_left_knee(), rviz_visual_tools::GREEN);

    Eigen::Affine3d text_pos {};
    text_pos.translation() = Eigen::Vector3d::UnitY() * 0.5;
    visualizer(text_pos, "score : " + std::to_string(s_.get_last()));
  }
};

#endif
