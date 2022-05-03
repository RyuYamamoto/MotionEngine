#ifndef _KINEMATIC_HPP_
#define _KINEMATIC_HPP_

#include "utils.hpp"

#include <cnoid/EigenUtil>
#include <cnoid/ExecutablePath>
#include <cnoid/JointPath>
#include <cnoid/Link>
#include <cnoid/SimpleController>

#include <iostream>

class Kinematics
{
public:
  Kinematics(cnoid::Body * body);
  ~Kinematics() = default;

  void calcForwaredKinematicsAll();
  void calcForwaredKinematics(const std::string end_effector_name);
  bool calcInverseKinematics(cnoid::Vector3 pos, cnoid::Vector3 rot, const std::string link_name);

  void getFootPos(
    cnoid::Vector3 & right_foot_pos, cnoid::Vector3 & right_foot_rot,
    cnoid::Vector3 & left_foot_pos, cnoid::Vector3 & left_foot_rot)
  {
    body_->calcForwardKinematics();
    right_foot_path_->calcForwardKinematics();
    left_foot_path_->calcForwardKinematics();

    right_foot_pos = right_foot_pos_;
    right_foot_rot = right_foot_rot_;
    left_foot_pos = left_foot_pos_;
    left_foot_rot = left_foot_rot_;
  }

  std::map<std::string, JointAngle> getRefAngle() { return ref_angle_; }
  void setRefAngle(std::map<std::string, JointAngle> ref_angle) { ref_angle_ = ref_angle; }

  void init();

  void setLinkName();

  cnoid::Link * link(const char * name) { return body_->link(name); }

private:
  cnoid::Body * body_;

  // end effector link info
  cnoid::Link * right_foot_;
  cnoid::Link * left_foot_;
  std::shared_ptr<cnoid::JointPath> right_foot_path_;
  std::shared_ptr<cnoid::JointPath> left_foot_path_;

  // end effector pose
  cnoid::Vector3 right_foot_pos_;
  cnoid::Vector3 right_foot_rot_;
  cnoid::Vector3 left_foot_pos_;
  cnoid::Vector3 left_foot_rot_;

  std::map<std::string, JointAngle> ref_angle_;
};

#endif
