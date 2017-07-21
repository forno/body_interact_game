#include <string>

#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <tf2_eigen/tf2_eigen.h>

#include "body_interact_game/body_interact_game.hpp"
#include "body_interact_game/visualizer.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "body_interact_game_node");
  ros::NodeHandle n;

  std::string root {"openni_coordinater"};
  int target_number {1};
  {
    ros::NodeHandle pn {"~"};
    pn.getParam("root", root);
    pn.getParam("num", target_number);
  }

  visualizer v {std::make_shared<rviz_visual_tools::RvizVisualTools>(root, root + std::to_string(target_number))};
  ros::Rate r {5};

  interact_game ig {n, root, static_cast<std::size_t>(target_number)};

  while (ros::ok()) {
    try {
      ig.update();
      ig.render(v);
      v.update();
    } catch (tf2::TransformException& e) {
      ROS_WARN_STREAM(e.what());
    }
    r.sleep();
  }
}
