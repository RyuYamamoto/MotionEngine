#include "MotionEngine.hpp"

MotionEngine::MotionEngine() {}

void MotionEngine::initialize(cnoid::SimpleControllerIO * io)
{
  body_ = io->body();
  dt_ = io->timeStep();

  for (std::size_t idx = 0; idx < body_->numJoints(); ++idx) {
    cnoid::Link * joint = body_->joint(idx);
    joint->setActuationMode(cnoid::Link::JointDisplacement);
    io->enableIO(joint);

    JointAngle joint_angle;
    joint_angle.id = joint->jointId();
    joint_angle.name = joint->jointName();
    joint_angle.ref_q = joint->q();
    joint_angle.prev_ref_q = joint_angle.ref_q;
    joint_angle.prev_q = joint_angle.ref_q;
    joint_angles_[joint_angle.name] = joint_angle;
  }

  kinematics_ = std::make_shared<Kinematics>(body_);
  kinematics_->calcForwaredKinematicsAll();
  kinematics_->setRefAngle(joint_angles_);
  kinematics_->getFootPos(right_foot_pos_, right_foot_rot_, left_foot_pos_, left_foot_rot_);
}

// motion command generate
void MotionEngine::receiveCommand(double * l_stick, double * r_stick)
{
  joystick_.readCurrentState();

  // get joy stick value[-1. ~ 1.]
  l_stick[0] = joystick_.getPosition(cnoid::Joystick::L_STICK_H_AXIS);
  l_stick[1] = joystick_.getPosition(cnoid::Joystick::L_STICK_V_AXIS);
  r_stick[0] = joystick_.getPosition(cnoid::Joystick::R_STICK_H_AXIS);
  r_stick[1] = joystick_.getPosition(cnoid::Joystick::R_STICK_V_AXIS);
}

// main controller
void MotionEngine::control()
{
  // get command from joy stick
  double l_stick[2], r_stick[2];
  receiveCommand(l_stick, r_stick);

  // TODO: ik demo
  left_foot_pos_.z() += (0.0001 * l_stick[0]);
  right_foot_pos_.z() += (0.0001 * l_stick[0]);

  //  kinematics_->setRefAngle(joint_angles_);
  bool result = false;
  if (
    kinematics_->calcInverseKinematics(right_foot_pos_, right_foot_rot_, "RLEG_JOINT5") and
    kinematics_->calcInverseKinematics(left_foot_pos_, left_foot_rot_, "LLEG_JOINT5"))
    joint_angles_ = kinematics_->getRefAngle();

  jointControl();
}

// torque controller by pd control
void MotionEngine::jointControl()
{
  for (std::size_t idx = 0; idx < body_->numJoints(); ++idx) {
    if (idx == 37) continue;  // for JAXON
    cnoid::Link * joint = body_->joint(idx);
    double q = joint->q();
    double dq = (q - joint_angles_[joint->name()].prev_q) / dt_;
    double ref_dq =
      (joint_angles_[joint->name()].ref_q - joint_angles_[joint->name()].prev_ref_q) / dt_;
    double u = (joint_angles_[joint->name()].ref_q - q) * p_gain[idx] + (ref_dq - dq) * d_gain[idx];
    // joint->u() = std::max(std::min(u, 500.0), -500.0);
    joint->q_target() = joint_angles_[joint->name()].ref_q;
    joint_angles_[joint->name()].prev_ref_q = joint_angles_[joint->name()].ref_q;
    joint_angles_[joint->name()].prev_q = q;
  }
}
