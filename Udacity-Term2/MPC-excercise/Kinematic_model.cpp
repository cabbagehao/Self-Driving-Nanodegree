// In this quiz you'll implement the global kinematic model.
#include <math.h>
#include <iostream>
#include "Dense"

//
// Helper functions
//
double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

const double Lf = 2;

// TODO: Implement the global kinematic model.
// Return the next state.
//
// NOTE: state is [x, y, psi, v]
// NOTE: actuators is [delta, a]
Eigen::VectorXd globalKinematic(Eigen::VectorXd state,
                                Eigen::VectorXd actuators, double dt) {
  Eigen::VectorXd next_state(state.size());

  //TODO complete the next_state calculation ...
  double x0 = state[0];
  double y0 = state[1];
  double phi = state[2];
  double v = state[3];
  
  double xt = x0 + v * dt * cos(phi);
  double yt = y0 + v * dt * sin(phi);
  double phi_t = phi + v/Lf * actuators[0] * dt;
  

//   double v_t_x = v * cos(phi) + actuators[1] * cos(actuators[0]);
//   double v_t_y = v * sin(phi) + actuators[1] * sin(actuators[0]);
//   double v_t = sqrt(v_t_x * v_t_x  + v_t_y * v_t_y);
  
  double v_t = v + actuators[1] * dt;
  next_state << xt, yt, phi_t, v_t;
  return next_state;
}

int main() {
  // [x, y, psi, v]
  Eigen::VectorXd state(4);
  // [delta, v]
  Eigen::VectorXd actuators(2);

  state << 0, 0, deg2rad(45), 1;
  actuators << deg2rad(5), 1;

  Eigen::VectorXd next_state = globalKinematic(state, actuators, 0.3);
  // should be [0.212132, 0.212132, 0.798488, 1.3]
  std::cout << next_state << std::endl;
}
