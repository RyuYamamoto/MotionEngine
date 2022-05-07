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

  FootStep foot_step;
  foot_step.foot_offset = 0.08;
  foot_step.max_step_x = 0.1;
  foot_step.max_step_y = 0.0;
  foot_step.max_step_yaw = 3.0;
  foot_step.step_num = 10;
  foot_step.period = 0.32;

  foot_step_pattern = planner.plan(foot_step);

#if 0
  std::ofstream ofs("foot.csv");
  ofs << "period,"
      << "ref zmp x,"
      << "ref zmp y" << std::endl;
  for (std::size_t idx = 0; idx < foot_step_pattern.size(); idx++) {
    ofs << idx * foot_step.period << "," << foot_step_pattern[idx](0) << ","
        << foot_step_pattern[idx][1] << std::endl;
  }
#endif
  return 0;
}
