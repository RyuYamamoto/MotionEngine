#ifndef _MOTION_ENGINE_HPP_
#define _MOTION_ENGINE_HPP_

#include "Kinematics.hpp"
#include "utils.hpp"

#include <cnoid/Joystick>

#include <iostream>

class MotionEngine
{
public:
  MotionEngine();

  void initialize(cnoid::SimpleControllerIO * io);

  void control();

  void jointControl();

  void receiveCommand(double * l_stick, double * r_stick);

private:
  std::shared_ptr<Kinematics> kinematics_;

  cnoid::Joystick joystick_;

  cnoid::Body * body_;

  double dt_;
  std::map<std::string, JointAngle> joint_angles_;

  cnoid::Vector3 right_foot_pos_;
  cnoid::Vector3 right_foot_rot_;
  cnoid::Vector3 left_foot_pos_;
  cnoid::Vector3 left_foot_rot_;
};

#endif
