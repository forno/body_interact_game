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
  {
    ros::NodeHandle pn {"~"};
    pn.getParam("root", root);
  }

  visualizer v {std::make_shared<rviz_visual_tools::RvizVisualTools>(root, "/rviz_visual_makers")};
  ros::Rate r {5};

  pose_generator_config pgc {0.9, 0.523599, 0.1, 0, 0.5, 0.349066, 0.1};
  interact_game ig {"openni_coordinater", 1, pgc, 30};

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
