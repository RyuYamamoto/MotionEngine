#ifndef _MOTION_ENGINE_HPP_
#define _MOTION_ENGINE_HPP_

#include "utils.hpp"
#include "Kinematics.hpp"

class MotionEngine
{
public:
  MotionEngine();

  void initialize(cnoid::SimpleControllerIO* io);

  void control();

  void jointControl();

private:
  std::shared_ptr<Kinematics> kinematics_;

  cnoid::Body* body_;

  double dt_;
  std::vector<JointAngle> joint_angles_;
};

#endif
