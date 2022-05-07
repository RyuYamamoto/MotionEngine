#include "FootStepPlanner.hpp"

#include <fstream>
#include <iostream>

FootStepPlanner::FootStepPlanner() {}

void FootStepPlanner::setFootStepParma(const FootStep param) { foot_step_ = param; }

void FootStepPlanner::setFootPos(
  const Eigen::Vector3f right_foot_pos, const Eigen::Vector3f left_foot_pos)
{
  right_foot_pose_ = right_foot_pos;
  left_foot_pose_ = left_foot_pos;
}

void FootStepPlanner::setTargetPos(const double x, const double y, const double yaw) {}

std::vector<Eigen::Vector2f> FootStepPlanner::plan(FootStep foot_step)
{
  foot_step_pattern_.clear();
  ref_zmp_pattern_.clear();

  // initial position
  ref_zmp_pattern_.emplace_back(Eigen::Vector2f::Zero());

  std::vector<Eigen::Vector3f> right_foot_pattern;
  std::vector<Eigen::Vector3f> left_foot_pattern;

  right_foot_pattern.emplace_back(right_foot_pose_);
  left_foot_pattern.emplace_back(left_foot_pose_);

  int support_leg = 0;
  for (int num = 0; num < foot_step.step_num; num++) {
    Eigen::Vector2f next_foot_pos;
    Eigen::Vector2f target_foot_pos;
    double zmp = (support_leg == 0 ? 1.0 : -1.0) * foot_step.foot_offset;
    next_foot_pos[0] = num * foot_step.max_step_x;
    next_foot_pos[1] = num * foot_step.max_step_y + zmp;
    target_foot_pos[0] = next_foot_pos[0] * std::cos(deg2rad(num * foot_step.max_step_yaw)) -
                         next_foot_pos[1] * std::sin(deg2rad(num * foot_step.max_step_yaw));
    target_foot_pos[1] = next_foot_pos[0] * std::sin(deg2rad(num * foot_step.max_step_yaw)) +
                         next_foot_pos[1] * std::cos(deg2rad(num * foot_step.max_step_yaw));
    support_leg = !support_leg;
    ref_zmp_pattern_.emplace_back(target_foot_pos);
  }

  // TODO: stop

  return ref_zmp_pattern_;
}
