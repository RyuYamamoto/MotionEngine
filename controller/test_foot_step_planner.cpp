#include "FootStepPlanner.hpp"

#include <fstream>

int main()
{
  FootStepPlanner planner;
  std::vector<Eigen::Vector2f> foot_step_pattern;

  Eigen::Vector3f right_foot_pos;
  Eigen::Vector3f left_foot_pos;

  right_foot_pos << 0.0, -0.08, 0.0;
  left_foot_pos << 0.0, 0.08, 0.0;

  planner.setFootPos(right_foot_pos, left_foot_pos);

  FootStepParam foot_step;
  foot_step.foot_offset = 0.08;
  foot_step.max_step_x = 0.1;
  foot_step.max_step_y = 0.0;
  foot_step.max_step_yaw = 3.0;
  foot_step.step_num = 5;
  foot_step.period = 0.32;

  foot_step_pattern = planner.plan(foot_step);

  return 0;
}
