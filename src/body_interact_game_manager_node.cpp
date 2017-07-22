#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <tf2_eigen/tf2_eigen.h>

#include "body_interact_game/pose_generator.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "body_interact_game_node");
  ros::NodeHandle n;
  ros::Publisher head_pub {n.advertise<geometry_msgs::Point>("head", 1)};
  ros::Publisher right_knee_pub {n.advertise<geometry_msgs::Point>("right_knee", 1)};
  ros::Publisher left_knee_pub {n.advertise<geometry_msgs::Point>("left_knee", 1)};
  ros::Publisher right_hand_pub {n.advertise<geometry_msgs::Point>("right_hand", 1)};
  ros::Publisher left_hand_pub {n.advertise<geometry_msgs::Point>("right_hand", 1)};

  pose_generator_config pgc {0.9, 0.523599, 0.1, 0, 0.5, 0.349066, 0.1};
  pose_generator pg {pgc};

  ros::Rate r {1. / 6};

  while (ros::ok()) {
    pg.update();
    head_pub.publish(tf2::toMsg(pg.get_head()));
    right_knee_pub.publish(tf2::toMsg(pg.get_right_knee()));
    left_knee_pub.publish(tf2::toMsg(pg.get_left_knee()));
    right_hand_pub.publish(tf2::toMsg(pg.get_right_hand()));
    left_hand_pub.publish(tf2::toMsg(pg.get_right_hand()));
    r.sleep();
  }
}
