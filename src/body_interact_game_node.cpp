#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <tf2_eigen/tf2_eigen.h>

#include "body_interact_game/body_analizer.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "body_interact_game_node");
  ros::NodeHandle n;
  body_analizer ba {"openni_coordinater", 1};
  rviz_visual_tools::RvizVisualTools rvt {"openni_coordinater", "/rviz_visual_makers"};
  ros::Rate r {5};

  while (ros::ok()) {
    try {
      ba.update();

      rvt.deleteAllMarkers();
      rvt.publishArrow(Eigen::Affine3d{Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), ba.get_head())},
                       rviz_visual_tools::BLUE, rviz_visual_tools::LARGE);
      rvt.publishArrow(Eigen::Affine3d{Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), ba.get_right_hand())},
                       rviz_visual_tools::BLUE, rviz_visual_tools::LARGE);
      rvt.publishArrow(Eigen::Affine3d{Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), ba.get_left_hand())},
                       rviz_visual_tools::BLUE, rviz_visual_tools::LARGE);
      rvt.publishArrow(Eigen::Affine3d{Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), ba.get_right_knee())},
                       rviz_visual_tools::BLUE, rviz_visual_tools::LARGE);
      rvt.publishArrow(Eigen::Affine3d{Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), ba.get_left_knee())},
                       rviz_visual_tools::BLUE, rviz_visual_tools::LARGE);

      rvt.trigger();
    } catch (tf2::TransformException& e) {
      ROS_WARN_STREAM(e.what());
    }
    r.sleep();
  }
}
