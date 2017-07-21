#include "body_interact_game/body_interact_game.hpp"

interact_game::interact_game(ros::NodeHandle& n, const std::string& root, std::size_t number)
  : ba_ {root, number},
    pr_ {n},
    s_ {}
{
}

void interact_game::update()
{
  ba_.update();
  //pr_.update();
  s_(ba_, pr_);
}
