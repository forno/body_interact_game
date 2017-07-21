#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <Eigen/Geometry>

class visualizer
{
  rviz_visual_tools::RvizVisualToolsPtr rvtp_;

public:
  visualizer(rviz_visual_tools::RvizVisualToolsPtr rvtp)
    : rvtp_ {rvtp}
  {
  }

  void operator()(Eigen::Vector3d vec, rviz_visual_tools::colors color) const
  {
    rvtp_->publishArrow(Eigen::Affine3d{Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), vec)},
                        color, rviz_visual_tools::LARGE);
  }

  void update() const
  {
    rvtp_->trigger();
    rvtp_->deleteAllMarkers();
  }
};

#endif
