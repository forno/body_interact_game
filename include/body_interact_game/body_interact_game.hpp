#ifndef BODY_INTERACT_GAME_H
#define BODY_INTERACT_GAME_H

#include <rviz_visual_tools/rviz_visual_tools.h>

#include "body_interact_game/body_analizer.hpp"
#include "body_interact_game/pose_receiver.hpp"
#include "body_interact_game/scorer.hpp"

#include <Eigen/Geometry>

class interact_game
{
  body_analizer ba_;
  pose_receiver pr_;
  scorer s_;

public:
  interact_game(ros::NodeHandle& n, const std::string& root, std::size_t number);

  void update();

  template<typename T>
  void render(T visualizer) const
  {
    visualizer(ba_.get_head(), rviz_visual_tools::BLUE);
    visualizer(ba_.get_right_knee(), rviz_visual_tools::BLUE);
    visualizer(ba_.get_left_knee(), rviz_visual_tools::BLUE);
    visualizer(pr_.get_head(), rviz_visual_tools::GREEN);
    visualizer(pr_.get_right_knee(), rviz_visual_tools::GREEN);
    visualizer(pr_.get_left_knee(), rviz_visual_tools::GREEN);

    Eigen::Affine3d text_pos {};
    text_pos.translation() = Eigen::Vector3d::UnitY() * 0.5;
    visualizer(text_pos, "score : " + std::to_string(s_.get_last()));
  }
};

#endif
