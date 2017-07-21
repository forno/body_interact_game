#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <tf2_eigen/tf2_eigen.h>

#include "body_interact_game/body_analizer.hpp"
#include "body_interact_game/pose_generator.hpp"
#include "body_interact_game/visualizer.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "body_interact_game_node");
  ros::NodeHandle n;
  rviz_visual_tools::RvizVisualToolsPtr rvtp {new rviz_visual_tools::RvizVisualTools{"openni_coordinater", "/rviz_visual_makers"}};
  visualizer v {rvtp};
  ros::Rate r {5};

  body_analizer ba {"openni_coordinater", 1};
  pose_generator_config pgc {0.9, 0.523599, 0.1, 0, 0.5, 0.349066, 0.1};
  pose_generator pg {pgc};

  std::size_t count {0};
  while (ros::ok()) {
    count = (count + 1) % 30; // update pg on every by 6 seconds
    if (!count)
      pg.update();
    try {
      ba.update();

      v(ba.get_head(), rviz_visual_tools::BLUE);
      v(ba.get_right_knee(), rviz_visual_tools::BLUE);
      v(ba.get_left_knee(), rviz_visual_tools::BLUE);
      v(pg.get_head(), rviz_visual_tools::GREEN);
      v(pg.get_right_knee(), rviz_visual_tools::GREEN);
      v(pg.get_left_knee(), rviz_visual_tools::GREEN);

      v.update();
    } catch (tf2::TransformException& e) {
      ROS_WARN_STREAM(e.what());
    }
    r.sleep();
  }
}
