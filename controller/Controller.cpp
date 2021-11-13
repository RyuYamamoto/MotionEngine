#include <cnoid/SimpleController>
#include <cnoid/Link>
#include <cnoid/ExecutablePath>

#include <iostream>
#include <fstream>

const double p_gain[] = {
  33000, 83000, 33000, 33000, 47000, 33000,
  33000, 83000, 33000, 33000, 47000, 33000,
  83000, 83000, 83000, 10000, 10000, 20000,
  20000, 20000, 20000, 20000, 20000, 20000,
  20000, 20000, 20000, 20000, 20000, 20000,
  20000, 20000, 20000, 400,   400,   400,
  400, 100
};

const double d_gain[] = {
  240, 240, 240, 240, 240, 240,
  240, 240, 240, 240, 240, 240,
  240, 240, 240, 200, 200, 320,
  320, 320, 320, 320, 320, 320,
  320, 320, 320, 320, 320, 320,
  320, 320, 320, 10,  10,  10,
  10,5
};

class Controller : public cnoid::SimpleController
{
public:
  virtual bool initialize(cnoid::SimpleControllerIO* io) override
  {
    body_ = io->body();
    dt_ = io->timeStep();

    for (std::size_t idx = 0; idx < body_->numJoints(); ++idx) {
      cnoid::Link* joint = body_->joint(idx);
      joint->setActuationMode(cnoid::Link::JointTorque);
      io->enableIO(joint);
      q_ref_.push_back(joint->q());
    }
    q_prev_ = q_ref_;

    return true;
  }
  virtual bool control() override
  {
    for (std::size_t idx = 0; idx < body_->numJoints(); ++idx) {
      if(idx == 37) continue;
      cnoid::Link* joint = body_->joint(idx);
      double q = joint->q();
      double dq = (q - q_prev_[idx]) / dt_;
      double u = (q_ref_[idx] - q) * p_gain[idx] + (0.0 - dq) * d_gain[idx];
      joint->u() = std::max(std::min(u, 500.0), -500.0);
      q_prev_[idx] = q;
    }
    return true;
  }

private:
  cnoid::Body* body_;

  std::vector<double> q_ref_;
  std::vector<double> q_prev_;

  double dt_;
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(Controller)
