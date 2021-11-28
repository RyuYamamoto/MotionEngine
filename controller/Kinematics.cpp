#include "Kinematics.hpp"

Kinematics::Kinematics(cnoid::Body* body)
  : body_(body)
{

}

void Kinematics::calcForwaredKinematics()
{
  body_->calcForwardKinematics();
}

bool Kinematics::calcInverseKinematics(cnoid::Vector3 pos, cnoid::Vector3 rot)
{
}
