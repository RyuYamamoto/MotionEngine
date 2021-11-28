#ifndef _KINEMATIC_HPP_
#define _KINEMATIC_HPP_

#include <cnoid/SimpleController>
#include <cnoid/Link>
#include <cnoid/ExecutablePath>
#include <cnoid/JointPath>

class Kinematics
{
public:
  Kinematics(cnoid::Body* body);

  void calcForwaredKinematics();
  bool calcInverseKinematics(cnoid::Vector3 pos, cnoid::Vector3 rot);

  cnoid::Link* link(const char* name) { return body_->link(name); }

private:
  cnoid::Body* body_;

  std::shared_ptr<cnoid::JointPath> right_foot_path_;
  std::shared_ptr<cnoid::JointPath> left_foot_path_;
};

#endif
