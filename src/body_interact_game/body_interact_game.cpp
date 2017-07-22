#include <std_msgs/Float64.h>

#include "body_interact_game/body_interact_game.hpp"

interact_game::interact_game(ros::NodeHandle& n, const std::string& root, std::size_t number)
  : ba_ {root, number},
    pr_ {n},
    s_ {},
    score_pub_ {n.advertise<std_msgs::Float64>("score_" + std::to_string(number), 1)}
{
}

void interact_game::update()
{
  ba_.update();
  //pr_.update();
  std_msgs::Float64Ptr msgp {new std_msgs::Float64{}};
  msgp->data = s_(ba_, pr_);
  score_pub_.publish(msgp);
}
