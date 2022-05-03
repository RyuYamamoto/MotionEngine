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

bool Kinematics::calComKinematics(
  cnoid::Vector3 ref_com, cnoid::Vector3 cur_com, const std::string link_name)
{
  int iteration = 100;
  double gain = 0.5;
  double error_eps = 1e-06;
  double lambda = 1.0e-12;

  cnoid::VectorXd dq(body_->numJoints());
  cnoid::VectorXd v(body_->numJoints());
  cnoid::MatrixXd J_com(3, body_->numJoints());

  bool result = false;

  for (int n = 0; n < iteration; n++) {
    if (ref_com.dot(ref_com) < error_eps) {
      result = true;
      break;
    }

    if (link_name == "RLEG_JOINT5")
      cnoid::calcCMJacobian(body_, right_foot_path_->baseLink(), J_com);
    else if (link_name == "LLEG_JOINT5")
      cnoid::calcCMJacobian(body_, left_foot_path_->baseLink(), J_com);

    cnoid::MatrixXd JJ =
      J_com * J_com.transpose() + lambda * cnoid::MatrixXd::Identity(J_com.rows(), J_com.rows());

    Eigen::ColPivHouseholderQR<cnoid::MatrixXd> QR;
    dq = J_com.transpose() * QR.compute(JJ).solve(ref_com - cur_com) * gain;
    for (std::size_t idx = 0; idx < body_->numJoints(); ++idx)
      body_->joint(idx)->q() += 0.3 * dq(idx);

    body_->calcForwardKinematics();
  }

  return result;
}
