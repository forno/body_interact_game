#include <body_interact_game/body_analizer.hpp>

body_analizer::body_analizer(std::string root, std::size_t target_number)
  : root_ {root},
    target_number_ {std::to_string(target_number)},
    base_name_ {"torso_" + target_number_},
    right_hand_name_ {"right_hand_" + target_number_},
    left_hand_name_ {"left_hand_" + target_number_},
    right_knee_name_ {"right_knee_" + target_number_},
    left_knee_name_ {"left_knee_" + target_number_}
{
}
