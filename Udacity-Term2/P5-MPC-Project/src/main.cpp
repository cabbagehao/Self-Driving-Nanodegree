#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {

          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          int pts_size = ptsx.size(); 
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double delta = j[1]["steering_angle"];
          double a = j[1]["throttle"];

          //rotate and shift the global waypoints ptsx and ptsy to the coordinate of vehicle
          for(int i = 0; i < pts_size; i++)
          {
            double x_diff = ptsx[i] - px;
            double y_diff = ptsy[i] - py;
            ptsx[i] = x_diff * cos(psi) + y_diff * sin(psi);
            ptsy[i] = y_diff * cos(psi) - x_diff * sin(psi);
          }

          Eigen::Map<Eigen::VectorXd> ptsx_new(&ptsx[0], pts_size);
          Eigen::Map<Eigen::VectorXd> ptsy_new(&ptsy[0], pts_size);

          // fit a polynomial and calculate the current cte/cpsi.
          Eigen::VectorXd coeffs = polyfit(ptsx_new, ptsy_new, 3);

          double cte = polyeval(coeffs, 0) - 0;
          double poly_slope =  coeffs[1]; // 3*coeffs[3] * px*px + 2*coeffs[2]*px +
          double epsi = 0 - atan(poly_slope) ;

          // Eigen::VectorXd state(6);
          // state << px, py, psi, v, cte, epsi;

          double latency_dt = 0.1;
          double Lf = 2.67;       //the distance between front wheel and center of gravity of vehicle

          //the state values of current position
          double cur_x = 0.0;
          double cur_y = 0.0;
          double cur_psi = 0.0;
          double cur_v = v;
          double cur_a = a;
          double cur_delta = delta;
          //the state values after latency
          double pred_x = cur_x + v*cos(cur_psi)*latency_dt;
          double pred_y = cur_y + v*sin(cur_psi)*latency_dt;
          double pred_psi = cur_psi - cur_v*cur_delta*latency_dt/Lf;
          double pred_v = cur_v + cur_a * latency_dt;
          double pred_cte = cte + cur_v*sin(epsi)*latency_dt;
          double pred_epsi = epsi - cur_v*cur_delta*latency_dt/Lf;
          //feed the predicted state into solver
          Eigen::VectorXd state(6);
          state << pred_x, pred_y, pred_psi, pred_v, pred_cte, pred_epsi;


          std::vector<vector <double> > vars = mpc.Solve(state, coeffs);

          vector<double> x_vals = vars[0];
          vector<double> y_vals = vars[1]; 
          x_vals.insert(x_vals.begin(), state[0]);       
          y_vals.insert(y_vals.begin(), state[1]);
          double steer_value =  vars[2][0];
          double throttle_value = vars[3][0];

          // cout << "x_vals.size() " << x_vals.size() <<endl  ;
          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          //if(abs(steer_value) > 0.3) steer_value = steer_value/4;
          // if(abs(steer_value / deg2rad(25) > 0.9)) steer_value = -steer_value;
          msgJson["steering_angle"] = steer_value / deg2rad(25);
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          // transform the x_vals and y_vals from global coordinates to vehicle coordinates.
          vector<double> mpc_x_vals = x_vals;
          vector<double> mpc_y_vals = y_vals;
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals = ptsx;
          vector<double> next_y_vals = ptsy;
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // // cout << "Send data: " << endl;
          // std::cout << msg << std::endl;
        
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
