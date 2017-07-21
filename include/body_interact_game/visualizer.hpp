#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <memory>
#include <utility>

#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <Eigen/Geometry>

class visualizer
{
  std::shared_ptr<rviz_visual_tools::RvizVisualTools> rvtp_;

public:
  visualizer(std::shared_ptr<rviz_visual_tools::RvizVisualTools> rvtp)
    : rvtp_ {std::move(rvtp)}
  {
  }

  void operator()(Eigen::Vector3d vec, rviz_visual_tools::colors color) const
  {
    rvtp_->publishArrow(Eigen::Affine3d{Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), vec)},
                        color, rviz_visual_tools::LARGE);
  }

  void operator()(Eigen::Affine3d pos, std::string s) const
  {
    rvtp_->publishText(pos, s, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  }

  void update() const
  {
    rvtp_->trigger();
    rvtp_->deleteAllMarkers();
  }
};

#endif
