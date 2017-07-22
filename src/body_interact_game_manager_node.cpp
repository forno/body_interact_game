#include <algorithm>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <tf2_eigen/tf2_eigen.h>

#include "body_interact_game/pose_generator.hpp"

class score_holder
{
  std::vector<double> scores_;
  std::vector<std::string> topics_;
  std::vector<ros::Subscriber> subs_ {};

public:
  score_holder(ros::NodeHandle n, const std::vector<std::string>& topics)
    : scores_(topics.size()),
      topics_ {topics}
  {
    for (std::size_t i {0}; i < topics.size(); ++i)
      subs_.emplace_back(n.subscribe<std_msgs::Float64>(topics[i], 1, [index = i, this](const std_msgs::Float64ConstPtr msgp) {
        scores_[index] = msgp->data;
      }));
  }

  double get_max_score() const noexcept
  {
    return *std::max_element(scores_.begin(), scores_.end());
  }

  std::string get_max_topic() const noexcept
  {
    return topics_[std::max_element(scores_.begin(), scores_.end()) - scores_.begin()];
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "body_interact_game_node");
  ros::NodeHandle n;
  ros::Publisher head_pub {n.advertise<geometry_msgs::Point>("head", 1)};
  ros::Publisher right_knee_pub {n.advertise<geometry_msgs::Point>("right_knee", 1)};
  ros::Publisher left_knee_pub {n.advertise<geometry_msgs::Point>("left_knee", 1)};
  ros::Publisher right_hand_pub {n.advertise<geometry_msgs::Point>("right_hand", 1)};
  ros::Publisher left_hand_pub {n.advertise<geometry_msgs::Point>("right_hand", 1)};

  std::vector<std::string> score_topics {};
  {
    ros::NodeHandle pn {"~"};
    pn.getParam("scores", score_topics);
  }

  if (score_topics.empty())
    return 0;

  score_holder sh {n, score_topics};
  pose_generator_config pgc {0.9, 0.523599, 0.1, 0, 0.5, 0.349066, 0.1};
  pose_generator pg {pgc};

  ros::Rate r {1. / 6};

  while (ros::ok()) {
    pg.update();
    ROS_INFO_STREAM("max score : " << sh.get_max_topic() << " / " << sh.get_max_score());
    head_pub.publish(tf2::toMsg(pg.get_head()));
    right_knee_pub.publish(tf2::toMsg(pg.get_right_knee()));
    left_knee_pub.publish(tf2::toMsg(pg.get_left_knee()));
    right_hand_pub.publish(tf2::toMsg(pg.get_right_hand()));
    left_hand_pub.publish(tf2::toMsg(pg.get_right_hand()));
    r.sleep();
  }
}
