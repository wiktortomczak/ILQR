#pragma once
#include "standardIncludes.h"
#include "ILQR.h"


class CartPole : public ILQR {

  const static double mc_;
  const static double mp_;
  const static double l_;
  const static double g_;

  // non-linear state dynamics
  virtual VectorXd f(const VectorXd &q, const VectorXd &u) {
    const double x_dot = q[1];
    const double theta = q[2];
    const double theta_dot = q[3];
    
    const double sin_theta = sin(theta);
    const double cos_theta = cos(theta);

    const double F = tanh(u[0]);
    const double temp = (F + mp_ * l_ * sqr(theta_dot) * sin_theta) / (mc_ + mp_);
    const double numerator = g_ * sin_theta - cos_theta * temp;
    const double denominator = l_ * (4.0 / 3.0 - mp_ * sqr(cos_theta) / (mc_ + mp_));
    const double theta_dot_dot = numerator / denominator;

    const double x_dot_dot = temp - mp_ * l_ * theta_dot_dot * cos_theta / (mc_ + mp_);

    VectorXd q_dot(4);
    q_dot[0] = x_dot;
    q_dot[1] = x_dot_dot;
    q_dot[2] = theta_dot;
    q_dot[3] = theta_dot_dot;
    
    return q_dot;
  }

  // non-linear cost function
  // total cost = lf(qf) + int l(q,u) dt
  virtual double l(const VectorXd &q, const VectorXd &u) {
    // Penalize large control inputs.
    return u.dot(u);
  }

  virtual double lf(const VectorXd &q) {
    // Penalize non-zero (target = 0) values of x, x_dot, theta, theta_dot.
    return 400 * q.dot(q);
  }
};


const double CartPole::mc_ = 1.0;
const double CartPole::mp_ = 0.1;
const double CartPole::l_ = 1.0;
const double CartPole::g_ = 9.80665;


double sqr(double x) { return x * x; }
