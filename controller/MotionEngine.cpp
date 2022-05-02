#include "MotionEngine.hpp"

MotionEngine::MotionEngine() {}

void MotionEngine::initialize(cnoid::SimpleControllerIO* io)
{
  body_ = io->body();
  dt_ = io->timeStep();

  for (std::size_t idx = 0; idx < body_->numJoints(); ++idx) {
    cnoid::Link* joint = body_->joint(idx);
    joint->setActuationMode(cnoid::Link::JointTorque);
    io->enableIO(joint);

    JointAngle joint_angle;
    joint_angle.id = joint->jointId();
    joint_angle.name = joint->jointName();
    joint_angle.ref_q = joint->q();
    joint_angle.prev_ref_q = joint_angle.ref_q;
    joint_angle.prev_q = joint_angle.ref_q;
    joint_angles_.emplace_back(joint_angle);
  }

  kinematics_ = std::make_shared<Kinematics>(body_);
  kinematics_->calcForwaredKinematics();
}

void MotionEngine::control()
{
  jointControl();
}

void MotionEngine::jointControl()
{
  for (std::size_t idx = 0; idx < body_->numJoints(); ++idx) {
    if(idx == 37) continue; // for JAXON
    cnoid::Link* joint = body_->joint(idx);
    double q = joint->q();
    double dq = (q - joint_angles_[idx].prev_q) / dt_;
    double ref_dq = (joint_angles_[idx].ref_q - joint_angles_[idx].prev_ref_q) / dt_;
    double u = (joint_angles_[idx].ref_q - q) * p_gain[idx] + (ref_dq - dq) * d_gain[idx];
    joint->u() = std::max(std::min(u, 500.0), -500.0);
    joint_angles_[idx].prev_ref_q = joint_angles_[idx].ref_q;
    joint_angles_[idx].prev_q = q;
  }
}
