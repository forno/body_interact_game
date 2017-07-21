#include "body_interact_game/body_interact_game.hpp"

interact_game::interact_game(const std::string& root, std::size_t number, const pose_generator_config& pgc, std::size_t interval)
  : ba_ {root, number},
    pg_ {pgc},
    interval_ {interval}
{
}

void interact_game::update()
{
  static std::size_t count {0};
  ba_.update();
  if (!(count = (count + 1) % 30)) {
    // TODO: Score calculate
    // TODO: Save the score
    pg_.update();
  }
}
