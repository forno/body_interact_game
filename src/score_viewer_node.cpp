#include <string>

#include <ros/ros.h>
#include <body_interact_game/Score.h>
#include <geometry_msgs/Pose.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "score_viewer_node");
  ros::NodeHandle n {};

  std::string root {"openni_depth_frame"};
  {
    ros::NodeHandle pn {"~"};
    pn.getParam("root", root);
  }

  rviz_visual_tools::RvizVisualTools rvt {root, "score_marker"};
  geometry_msgs::Pose pos;
  pos.position.z = 1;
  ros::Subscriber s {n.subscribe<body_interact_game::Score>("max_score", 1, [&rvt, pos](body_interact_game::ScoreConstPtr msgp){
    rvt.publishText(pos, msgp->name + '\n' + std::to_string(msgp->score), rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
    rvt.trigger();
  })};

  ros::spin();
}
