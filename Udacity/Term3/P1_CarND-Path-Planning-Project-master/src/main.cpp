#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "helper.h"

using namespace std;
// for convenience
using json = nlohmann::json;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  // Detect distance in the front.
  double detect_s_front = 42;
  // Detect distance at behind.
  double detect_s_behind = 30;
  // start velocity.
  double ref_vel = 0;
  // velocity chanage rate. km/h
  double cosy_acc = 1.2;
  // start in lane 1
  int lane = 1;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &ref_vel, &lane,&max_s, &detect_s_front, &cosy_acc](
                  uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
            // j[1] is the data JSON object
        	  // Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"]; // km/h
            lane = car_d / 4;
          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

            int prev_size = previous_path_x.size();
          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];
            /* 找到当前车道最近的一辆车。 */
            bool too_close = false;            
            int front_car_id = 0;
            double min_delta_s = detect_s_front;

            for(int i = 0; i < sensor_fusion.size(); i++)
            {
              double d = sensor_fusion[i][6];
              double check_car_s = sensor_fusion[i][5];
              double delta_s;

              if(d < (2+4*lane+2) && d > (2+4*lane-2) )
              {
                delta_s = check_car_s - car_s;

                // 在圆圈终点时特殊处理。 
                if(car_s > max_s - detect_s_front && check_car_s < detect_s_front) 
                {
                  delta_s += max_s;
                }

                if(delta_s > 0 && delta_s < min_delta_s)
                {
                  front_car_id = i;
                  min_delta_s = delta_s;
                }
              
              }
            }

            /* 若前方最近的一辆车速度比本车慢，则采取相应操作 */
            auto front_car = sensor_fusion[front_car_id];
            double vx = front_car[3];
            double vy = front_car[4]; 
            double front_car_s = front_car[5];
            double front_speed = sqrt(vx*vx+vy*vy); // m/s to km/h
            //front_car_s += (double)prev_size*0.02*front_speed;
            if(min_delta_s < detect_s_front && min_delta_s > 0 ) 
            {
              too_close = true;
              cout << "Detect a car in front of " << min_delta_s << "m." << endl;
              
              vector< vector<int> > next_lane = get_next_lane_and_vehicle(sensor_fusion, car_d, car_s);
              // check if it is safety to change lane.
              int best_next_lane_id = get_safety_next_lane(sensor_fusion, next_lane, car_s, car_speed);
              if(best_next_lane_id != -1) // It's safety to change lane, do it.
              {
                lane = best_next_lane_id;
              }
              else          // It's not safety to change lane, keep lane and low speed.
              {
                if(front_speed < car_speed)
                {
                  ref_vel = car_speed - cosy_acc;
                }
                else
                {
                  ref_vel = car_speed + cosy_acc;
                }
              }
              
            }

            // There is another way to slow down. we can shorten the predict point if the is a car in front of us.
            if(!too_close && ref_vel < 48.8)
            {
              ref_vel += cosy_acc;
              cout << "ref_vel: " << ref_vel <<" " << endl;
            }


            auto next_vals = generate_trajectory(car_x, car_y, car_s, car_yaw, lane, ref_vel, 
                                                  previous_path_x, previous_path_y, prev_size, 
                                                  map_waypoints_s, map_waypoints_x, map_waypoints_y);

            json msgJson;
          	msgJson["next_x"] = next_vals[0];
          	msgJson["next_y"] = next_vals[1];
          

            

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
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
