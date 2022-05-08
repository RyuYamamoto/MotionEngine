#include "FootStepPlanner.hpp"

#include <fstream>
#include <iostream>

FootStepPlanner::FootStepPlanner() {}

void FootStepPlanner::setFootStepParma(const FootStepParam param) { foot_step_ = param; }

void FootStepPlanner::setFootPos(
  const Eigen::Vector3f right_foot_pos, const Eigen::Vector3f left_foot_pos)
{
  right_foot_pose_ = right_foot_pos;
  left_foot_pose_ = left_foot_pos;
}

void FootStepPlanner::setTargetPos(const double x, const double y, const double yaw) {}

std::vector<Eigen::Vector2f> FootStepPlanner::plan(FootStepParam foot_step_param)
{
  foot_step_pattern_.clear();
  ref_zmp_pattern_.clear();

  // initial position
  ref_zmp_pattern_.emplace_back(Eigen::Vector2f::Zero());

  int support_leg = 0;

  foot_step_pattern_.clear();

  FootStep foot_step;
  foot_step.left_foot_pos = left_foot_pose_;
  foot_step.right_foot_pos = right_foot_pose_;
  foot_step.leg = support_leg;
  foot_step_pattern_.emplace_back(foot_step);

  for (int num = 0; num < foot_step_param.step_num; num++) {
    Eigen::Vector2f next_foot_pos;
    Eigen::Vector2f target_foot_pos;
    double zmp = (support_leg == 0 ? 1.0 : -1.0) * foot_step_param.foot_offset;
    next_foot_pos[0] = num * foot_step_param.max_step_x;
    next_foot_pos[1] = num * foot_step_param.max_step_y + zmp;
    target_foot_pos[0] = next_foot_pos[0] * std::cos(deg2rad(num * foot_step_param.max_step_yaw)) -
                         next_foot_pos[1] * std::sin(deg2rad(num * foot_step_param.max_step_yaw));
    target_foot_pos[1] = next_foot_pos[0] * std::sin(deg2rad(num * foot_step_param.max_step_yaw)) +
                         next_foot_pos[1] * std::cos(deg2rad(num * foot_step_param.max_step_yaw));
    if (support_leg == 0) {
      left_foot_pose_[0] = target_foot_pos[0];
      left_foot_pose_[1] = target_foot_pos[1];
      left_foot_pose_[2] = num * foot_step_param.max_step_yaw;
    } else {
      right_foot_pose_[0] = target_foot_pos[0];
      right_foot_pose_[1] = target_foot_pos[1];
      right_foot_pose_[2] = num * foot_step_param.max_step_yaw;
    }
    foot_step.yaw = num * foot_step_param.max_step_yaw;
    foot_step.left_foot_pos = left_foot_pose_;
    foot_step.right_foot_pos = right_foot_pose_;
    foot_step.leg = support_leg;
    foot_step_pattern_.emplace_back(foot_step);
    support_leg = !support_leg;
    ref_zmp_pattern_.emplace_back(target_foot_pos);
  }

  // TODO: stop
  auto last_zmp = ref_zmp_pattern_.end();
  Eigen::Vector2f stop_zmp(Eigen::Vector2f::Zero());
  stop_zmp[0] = (foot_step_param.step_num-1) * foot_step_param.max_step_x;
  stop_zmp[1] = (foot_step_param.step_num-1) * foot_step_param.max_step_y;
  ref_zmp_pattern_.emplace_back(stop_zmp);

  // TODO: output csv
  std::ofstream ofs("ref_zmp.csv");
  ofs << "ref zmp x," << "ref zmp y"<< std::endl;
  for(auto zmp : ref_zmp_pattern_)
    ofs << zmp(0) << "," << zmp(1) << std::endl;

  std::ofstream ofs1("foot_step.csv");
  ofs1 << "right_foot_pos_x," << "right_foot_pos_y," << "left_foot_pos_x," << "left_foot_pos_y," << "yaw," << "sup_leg" << std::endl;
  for(auto foot : foot_step_pattern_)
    ofs1 << foot.right_foot_pos.x() << ","
        << foot.right_foot_pos.y() << ","
        << foot.left_foot_pos.x() << ","
        << foot.left_foot_pos.y() << ","
        << foot.yaw << ","
        << foot.leg << std::endl;

  return ref_zmp_pattern_;
}
