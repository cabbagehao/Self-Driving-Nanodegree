#include <sstream>


using namespace std;

// The max s value before wrapping around the track back to 0
double max_s = 6945.554;

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
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for(int i = 0; i < maps_x.size(); i++)
    {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x,y,map_x,map_y);
        if(dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }

    }

    return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

    int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2( (map_y-y),(map_x-x) );

    double angle = abs(theta-heading);

    if(angle > pi()/4)
    {
        closestWaypoint++;
    }

    return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
    int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

    int prev_wp;
    prev_wp = next_wp-1;
    if(next_wp == 0)
    {
        prev_wp  = maps_x.size()-1;
    }

    double n_x = maps_x[next_wp]-maps_x[prev_wp];
    double n_y = maps_y[next_wp]-maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;

    double frenet_d = distance(x_x,x_y,proj_x,proj_y);

    //see if d value is positive or negative by comparing it to a center point

    double center_x = 1000-maps_x[prev_wp];
    double center_y = 2000-maps_y[prev_wp];
    double centerToPos = distance(center_x,center_y,x_x,x_y);
    double centerToRef = distance(center_x,center_y,proj_x,proj_y);

    if(centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for(int i = 0; i < prev_wp; i++)
    {
        frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
    }

    frenet_s += distance(0,0,proj_x,proj_y);

    return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
    int prev_wp = -1;

    while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
    {
        prev_wp++;
    }

    int wp2 = (prev_wp+1)%maps_x.size();

    double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s-maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
    double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

    double perp_heading = heading-pi()/2;

    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);

    return {x,y};

}

vector< vector<int> > get_next_lane_and_vehicle(vector<vector<double> > const& sensor_fusion, int car_d, int car_s)
{
  bool is_safety;
  // Find the next lane.
  vector< vector<int> >next_lanes;
  vector<int> temp;
  vector<int> temp2;
  int LANE_NUM = 3;
  int LANE_WIDTH = 4;
  int current_lane = car_d / LANE_WIDTH;
  if(current_lane > 0)
  {
    temp.push_back(current_lane - 1);
    next_lanes.push_back(temp);
  }
  if(current_lane < LANE_NUM-1)
  {
    temp2.push_back(current_lane + 1);
    next_lanes.push_back(temp2);
  }
  // cout << "after next_lanes push_back: " << next_lanes.size() << endl;
  // Find the next lanes' nearest cars at behind and in the front.
  for(int j = 0; j < next_lanes.size(); j++) 
  {
    double min_s_front = 10000;
    double max_s_behind = -1;
    int next_lane_behind_id = -1;
    int next_lane_front_id = -1;
    for(int i = 0; i < sensor_fusion.size(); i++)
    {
      double d = sensor_fusion[i][6];

      if(d < (2+4*next_lanes[j][0]+2) && d > (2+4*next_lanes[j][0]-2) )
      {
        // double vx = sensor_fusion[i][3];
        // double vy = sensor_fusion[i][4]; 
        // double check_speed = sqrt(vx*vx+vy*vy);
        double check_car_s = sensor_fusion[i][5];
        if(check_car_s > car_s && check_car_s < min_s_front)
        {
          next_lane_front_id = i;
          min_s_front = check_car_s;
        } 
        if(check_car_s < car_s && check_car_s > max_s_behind)
        {
          next_lane_behind_id = i;
          max_s_behind = check_car_s;
        }                      
      }
    }
    next_lanes[j].push_back(next_lane_front_id);
    next_lanes[j].push_back(next_lane_behind_id);
  } 

  return next_lanes;
}  

int get_safety_next_lane(vector<vector<double> > const& sensor_fusion, vector< vector<int> >next_lanes, double car_s, double car_speed)
{
  // cout << "In get_safety_next_lane " << next_lanes.size() <<endl;
  double next_lane_speed = 0;
  double best_next_lane_id = -1;

  for(int i = 0; i < next_lanes.size(); i++)
  {  
      double next_front_s;
      double vx_front;
      double vy_front;
      double next_behind_s;
      double next_front_speed;

      int next_behind_car_id = next_lanes[i][2];
      if(next_behind_car_id == -1)
      {
        next_behind_s = -1;
      }
      else
      {
        vector<double> next_behind_car = sensor_fusion[next_behind_car_id];
        next_behind_s = sensor_fusion[next_behind_car_id][5];
        // double vx_behind = sensor_fusion[next_behind_car_id][3];
        // double vy_behind = sensor_fusion[next_behind_car_id][4];                  
        // double next_behind_speed = sqrt(vx_behind*vx_behind + vy_behind*vy_behind);
      }
      
      // 判断相邻车道前后方没有车的情况。
      int next_front_car_id = next_lanes[i][1];
      if(next_front_car_id == -1)
      { 
        // 前方没有车，后方有车，这条道不能切换
        if(next_behind_car_id != -1 && car_s - next_behind_s < 10)
        {
          cout << "lane " << next_lanes[i][0] << " behind car is too close." << endl;
          continue;
        }
        else  // 前后都没有车，直接选取这条道。
        {
          best_next_lane_id = next_lanes[i][0];
          cout << "lane " << best_next_lane_id << " is safety to change.." << endl;
          break;
        }
        next_front_speed = 1000;
      }
      else
      {
        vector<double> next_front_car = sensor_fusion[next_front_car_id];
        next_front_s = next_front_car[5];
        vx_front = next_front_car[3];
        vy_front = next_front_car[4];
        next_front_speed = sqrt(vx_front*vx_front + vy_front*vy_front);
      }      

      // 当前有足够间距 && next_front_speed > car_speed  && 
      if( next_front_s >= car_s + 30 && next_behind_s <= car_s-10)
      {

          // 由于邻车道车速较快，所以考虑变道后 后方车辆是否可能碰撞。
          // 如果当前邻车道速度比另一相邻车道速度快，则change到当前邻车道。
          //double delta_behind_s = car_s - next_behind_s;
          //double delta_behind_v = car_speed - next_behind_speed;
          // delta_behind_s >= delta_behind_v * 5 &&
          if( next_front_speed > next_lane_speed)
          {
            best_next_lane_id = next_lanes[i][0];
            next_lane_speed = next_front_speed;   // 如果有2个相邻车道可选择，这里能保证优先切换到速度最快的车道。
          }

          cout << "lane " << best_next_lane_id << "is safety to change." << endl;
       
      }
      else
      {
        string str; 
        stringstream stream; 
        stream << next_lanes[i][0];
        stream >> str;
        string reason = "lane " + str + " is NOT safety to change. Because";
        if (next_front_s >= car_s + 30) reason += " front";
        if (next_front_s >= car_s + 30 && next_behind_s <= car_s-10) reason += " and";
        if (next_behind_s <= car_s-10) reason += " behind";
        reason += " car is too close." ;
        cout << reason << endl;
      }
  }

  return best_next_lane_id;
}

void get_prevpath_last_2_points(vector<double>& ptsx, vector<double>& ptsy, double &ref_x, double &ref_y,  double car_x, double car_y, 
                                    double car_yaw, vector<double> previous_path_x, vector<double> previous_path_y, int prev_size)
{
    // pts 添加 previous_path的后两个值。 如果没有则计算生成。
    if(prev_size < 2)
    {
      double prev_car_x = car_x - cos(car_yaw);
      double prev_car_y = car_y - sin(car_yaw);

      ptsx.push_back(prev_car_x);
      ptsx.push_back(car_x);

      ptsy.push_back(prev_car_y);
      ptsy.push_back(car_y);
    }
    else
    {
      ref_x = previous_path_x[prev_size-1];
      ref_y = previous_path_y[prev_size-1];

      double ref_x_prev = previous_path_x[prev_size-2];
      double ref_y_prev = previous_path_y[prev_size-2];

      ptsx.push_back(ref_x_prev);
      ptsx.push_back(ref_x);

      ptsy.push_back(ref_y_prev);
      ptsy.push_back(ref_y);

    }    
}

void get_some_anchors(vector<double>& ptsx, vector<double>& ptsy, double car_s, int lane, 
                        vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y)
{
    // pts 添加 3个锚点。用于 spline 计算
    std::vector<double> next_wp0 = getXY(car_s+55, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    std::vector<double> next_wp1 = getXY(car_s+65, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    std::vector<double> next_wp2 = getXY(car_s+75, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);
}


void transform_coordinates(vector<double>& ptsx, vector<double>& ptsy, double ref_yaw, double &ref_x, double &ref_y)
{
    // 转换坐标系到car当前坐标系
    for (int i = 0; i < ptsx.size(); i++ )
    {
      double shift_x = ptsx[i] - ref_x;
      double shift_y = ptsy[i] - ref_y;

      ptsx[i] = shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw);
      ptsy[i] = shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw);

    }  
}

vector<vector<double> > generate_points_by_spline(vector<double> previous_path_x, vector<double> previous_path_y, int prev_size, 
                                        vector<double> ptsx, vector<double> ptsy, double ref_vel, double ref_yaw, double ref_x, double ref_y)
{

    // Define the acutal (x,y) points we will use for the planner
    vector<double> next_x_vals;
    vector<double> next_y_vals;    
    // 将上次previous_path剩余的值直接填充到下一次预测中
    for(int i = 0; i < prev_size; i++)
    {
      next_x_vals.push_back(previous_path_x[i]);
      next_y_vals.push_back(previous_path_y[i]);
    }


    // create a spline
    tk::spline s;
    // set(x,y) points to the spline
    s.set_points(ptsx, ptsy);

    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist =sqrt(target_x*target_x + target_y*target_y);

    double x_add_on = 0;
    // 当previous_path不足50个点时，用spline通过ptsx, ptsy计算出预测点
    for (int i = 1; i <= 50-prev_size; i++)
    {
      double N = (target_dist / (0.02*ref_vel / 2.24)); // miles/h -> m/s
      double x_point = x_add_on + target_x / N;
      //cout << "x_add_on: " << x_add_on << " " << x_point << endl;
      double y_point = s(x_point);

      x_add_on = x_point;

      double x_ref = x_point;
      double y_ref = y_point;
      x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
      y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);

      x_point += ref_x;
      y_point += ref_y;

      next_x_vals.push_back(x_point);
      next_y_vals.push_back(y_point);
    }    

    vector<vector<double> > next_vals;
    next_vals.push_back(next_x_vals);
    next_vals.push_back(next_y_vals);
    return next_vals;
}

vector<vector<double> > generate_trajectory(double car_x, double car_y, double car_s, double car_yaw, double lane, double ref_vel,
                                            vector<double> previous_path_x, vector<double> previous_path_y, int prev_size, 
                                            vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y)
{
    vector<double> ptsx;
    vector<double> ptsy;

    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = deg2rad(car_yaw);

    vector<vector<double> > trajectory_points;


    get_prevpath_last_2_points(ptsx, ptsy, ref_x, ref_y, car_x, car_y, car_yaw, previous_path_x, previous_path_y, prev_size);

    get_some_anchors(ptsx, ptsy, car_s, lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

    transform_coordinates(ptsx, ptsy, ref_yaw, ref_x, ref_y);

    trajectory_points = generate_points_by_spline(previous_path_x, previous_path_y, prev_size, ptsx, ptsy, ref_vel, ref_yaw, ref_x, ref_y);

    return trajectory_points;
}