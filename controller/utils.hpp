#ifndef _UTILS_HPP_
#define _UTILS_HPP_

#include <string>

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

struct JointAngle
{
  int id;
  std::string name;
  double ref_q;
  double prev_ref_q;
  double prev_q;
};

#endif