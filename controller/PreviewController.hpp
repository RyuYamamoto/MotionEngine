#ifndef _PREVIEWCONTROLLER_H_
#define _PREVIEWCONTROLLER_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include <stdio.h>

#include <iostream>
#include <vector>

using namespace std;
using namespace Eigen;

const double ACCELERATION_OF_GRAVITY = -9.810;

class preview_control
{
protected:
  Matrix<double, 3, 3> A;
  Matrix<double, 3, 1> b;
  Matrix<double, 1, 3> c;
  Matrix<double, 3, 2> xk;
  Matrix<double, 1, 2> u, p;
  vector<double> fi;
  const double dt, preview_delay, zc, stop_time;
  int preview_num, foot_step_num;

private:
  Matrix<double, 3, 3> P;
  Matrix<double, 1, 3> K;
  Matrix<double, 2, 1> temp_refzmp;
  const double Q, R;

public:
  vector<Vector2d> refzmp;

public:
  preview_control(
    const double _dt, const double _preview_delay, const double _zc, const double _Q = 1e+08,
    const double _R = 1.0)
  : dt(_dt),
    preview_delay(_preview_delay),
    zc(_zc),
    Q(_Q),
    R(_R),
    preview_num(0),
    stop_time(1.0),
    xk(Matrix<double, 3, 2>::Zero())
  {
    A << 1, dt, (dt * dt) / 2, 0, 1, dt, 0, 0, 1;
    b << (dt * dt * dt) / 6.0, (dt * dt) / 2, dt;
    c << 1, 0, zc / ACCELERATION_OF_GRAVITY;

    calc_riccati_equation<3>(A, b, c);

    calc_f();
  }
  ~preview_control() {}
  template <size_t dim>
  bool calc_riccati_equation(
    Matrix<double, dim, dim> A, Matrix<double, dim, 1> b, Matrix<double, 1, dim> c)
  {
    Matrix<double, dim, dim> P_pre(Matrix<double, dim, dim>::Zero());
    const int MAX_ITERATION = 10000;

    for (int i = 0; i < MAX_ITERATION; i++) {
      K = (1.0 / (R + b.transpose() * P * b)) * b.transpose() * P * A;
      P_pre = A.transpose() * P * A + c.transpose() * Q * c - A.transpose() * P * b * K;
      if ((abs((P - P_pre).array()) < 1e-10).all()) {
        P = P_pre;
        K = (1.0 / (R + b.transpose() * P * b)) * b.transpose() * P * A;
        return true;
      }
      P = P_pre;
    }
    return false;
  }
  void buffer_clear()
  {
    refzmp.clear();
    preview_num = 0;
  }
  void interpolation_zmp_trajectory(vector<Vector3d> foot_step_list);
  void set_com_param(Vector2f & com_pos, Vector2f & com_vel, Vector2f & com_acc);
  void get_ref_zmp(Matrix<double, 2, 1> & refzmp) { refzmp = this->temp_refzmp; }
  void output_zmp(Matrix<double, 1, 2> & output_zmp) { output_zmp = this->p; }
  void calc_f();
  void calc_u();
  void calc_xk(Vector2d & com_pos, Vector2d & com_vel, Vector2d & com_acc);
  bool update(Vector2d & com_pos, Vector2d & com_vel, Vector2d & com_acc);
};

#endif
