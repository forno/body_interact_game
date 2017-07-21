#ifndef SCORER_H
#define SCORER_H

class scorer
{
  double last_score_ {0};

public:
  scorer() noexcept = default;

  template<typename T, typename U>
  double operator()(const T& target, const U& base)
  {
    last_score_ = 0;
    last_score_ += target.get_head().normalized().dot(base.get_head().normalized());
    last_score_ += target.get_right_knee().normalized().dot(base.get_right_knee().normalized());
    last_score_ += target.get_left_knee().normalized().dot(base.get_left_knee().normalized());
    return last_score_;
  }

  double get_last() const noexcept
  {
    return last_score_;
  }
};

#endif
