
#include "standardIncludes.h"
#include "cartpole.h"
#include <fstream>

int main() {
  CartPole cartpole;

  // state: x, x_dot, theta, theta_dot
  Vector4d startState(0, 0, pi/4, 0);
  Vector4d endState(0, 0, 0, 0); 
  const int K = 100; // length of trajectory
  timeDelta = 0.05;
  cartpole.generateFeedbackController(startState, endState, K, 1);

  ofstream ofs("cartpole.trajectory.csv", ios::out);
  ofs << "x, x_dot, theta, theta_dot, u" << endl;
  for (int k = 0; k < K; ++k) {  // TODO
    const double x = cartpole.xs[k][0];
    const double x_dot = cartpole.xs[k][1];
    const double theta = cartpole.xs[k][2];
    const double theta_dot = cartpole.xs[k][3];
    if (k < K - 1) {
      const double u = cartpole.us[k][0];
      ofs << x << ", " << x_dot << ", " << theta << ", " << theta_dot << ", " << u << endl;
    } else {
      ofs << x << ", " << x_dot << ", " << theta << ", " << theta_dot << endl;
    }
  }

  return 0;
}
