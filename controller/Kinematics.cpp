#include "Kinematics.hpp"

Kinematics::Kinematics(cnoid::Body * body) : body_(body) { init(); }

void Kinematics::calcForwaredKinematicsAll() { body_->calcForwardKinematics(); }

void Kinematics::calcForwaredKinematics(const std::string end_effector_name)
{
  if (end_effector_name == "RLEG_JOINT5")
    right_foot_path_->calcForwardKinematics();
  else if (end_effector_name == "LLEG_JOINT5")
    left_foot_path_->calcForwardKinematics();
}

void Kinematics::init()
{
  right_foot_ = this->link("RLEG_JOINT5");
  left_foot_ = this->link("LLEG_JOINT5");

  cnoid::Link * root = body_->rootLink();
  right_foot_path_ = cnoid::JointPath::getCustomPath(body_, root, right_foot_);
  left_foot_path_ = cnoid::JointPath::getCustomPath(body_, root, left_foot_);

  // update current pose
  right_foot_path_->calcForwardKinematics();
  left_foot_path_->calcForwardKinematics();
  // get end effector pose
  right_foot_pos_ = right_foot_->p();
  right_foot_rot_ = cnoid::rpyFromRot(right_foot_->attitude());
  left_foot_pos_ = left_foot_->p();
  left_foot_rot_ = cnoid::rpyFromRot(left_foot_->attitude());
}

bool Kinematics::calcInverseKinematics(
  cnoid::Vector3 pos, cnoid::Vector3 rot, const std::string link_name)
{
  bool result = false;

  if (link_name == "RLEG_JOINT5") {
    result = right_foot_path_->calcInverseKinematics(
      pos, right_foot_->calcRfromAttitude(cnoid::rotFromRpy(rot)));
    for (std::size_t idx = 0; idx < right_foot_path_->numJoints(); ++idx) {
      cnoid::Link * joint = right_foot_path_->joint(idx);
      ref_angle_[joint->name()].ref_q = joint->q();
    }
  } else if (link_name == "LLEG_JOINT5") {
    result = left_foot_path_->calcInverseKinematics(
      pos, left_foot_->calcRfromAttitude(cnoid::rotFromRpy(rot)));

    for (std::size_t idx = 0; idx < left_foot_path_->numJoints(); ++idx) {
      cnoid::Link * joint = left_foot_path_->joint(idx);
      ref_angle_[joint->name()].ref_q = joint->q();
    }
  }
  body_->calcForwardKinematics();

  return result;
}
