#ifndef _FOOT_STEP_PLANNER_HPP_
#define _FOOT_STEP_PLANNER_HPP_

#include <Eigen/Dense>

#include <vector>

struct FootStep
{
  int step_num;
  double max_step_x;
  double max_step_y;
  double max_step_yaw;
  double period;
  double foot_offset;
};

class FootStepPlanner
{
public:
  FootStepPlanner();
  ~FootStepPlanner() = default;

  inline double deg2rad(double degree) { return degree * M_PI / 180.0; }

  void setFootStepParma(const FootStep param);
  void setFootPos(const Eigen::Vector3f right_foot_pos, const Eigen::Vector3f left_foot_pos);

  void setTargetPos(const double x, const double y, const double yaw);
  std::vector<Eigen::Vector2f> plan(FootStep foot_step);

  std::vector<Eigen::Vector2f> getRefZMP() { return ref_zmp_pattern_; }
  std::vector<Eigen::Vector3f> getFootStep() { return foot_step_pattern_; }

private:
  FootStep foot_step_;

  Eigen::Vector3f right_foot_pose_;
  Eigen::Vector3f left_foot_pose_;

  std::vector<Eigen::Vector2f> ref_zmp_pattern_;
  std::vector<Eigen::Vector3f> foot_step_pattern_;
};

#endif
