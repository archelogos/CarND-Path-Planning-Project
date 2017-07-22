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
#include "vehicle.h"
#include "road.h"

using namespace std;
using namespace tk; // spline

// for convenience
using json = nlohmann::json;

// PROJECT LIMIT CONSTANTS
const double MAX_SPEED = 50.0; // mph/h
const double MAX_ACCEL= 10.0; // m/s^2
const double MAX_JERK = 10.0; // m/s^3

// OTHER CONSTANTS
const double SAFETY_DISTANCE = 50.0; // m

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double mph2ms(double v) { return v * (1600.0/3600.0); }
double ms2mph(double v) { return v * (3600.0/1600.0); }

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
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
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

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
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
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
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
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
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

/* Helpers */
enum LANE { LEFT, CENTER, RIGHT };
LANE getLane(double d) {
  LANE lane;
  if (d < 4.0) {
    lane = LEFT;
  }
  else if ((d >= 4.0) && (d < 8.0)) {
    lane = CENTER;
  }
  else {
    lane = RIGHT;
  }
  return lane;
}


/* Main */

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

  // define wp spline trajectory
  spline wp_spline_x;
  spline wp_spline_y;
  spline wp_spline_dx;
  spline wp_spline_dy;

  // set points
  wp_spline_x.set_points(map_waypoints_s, map_waypoints_x);
  wp_spline_y.set_points(map_waypoints_s, map_waypoints_y);
  wp_spline_dx.set_points(map_waypoints_s, map_waypoints_dx);
  wp_spline_dy.set_points(map_waypoints_s, map_waypoints_dy);

  // PathPlanner instance
  // PathPlanner pathPlanner;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&wp_spline_x,&wp_spline_y,&wp_spline_dx,&wp_spline_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
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
            double car_speed = j[1]["speed"];

            // Previous path data given to the Planner
            auto previous_path_x = j[1]["previous_path_x"];
            auto previous_path_y = j[1]["previous_path_y"];
            // Previous path's end s and d values
            double end_path_s = j[1]["end_path_s"];
            double end_path_d = j[1]["end_path_d"];

            // Sensor Fusion Data, a list of all other cars on the same side of the road.
            auto sensor_fusion = j[1]["sensor_fusion"];

            json msgJson;

            vector<double> next_x_vals;
            vector<double> next_y_vals;

            /* CORE */

            /*****************
            ******CONTEXT*****
            *****************/

            /* Step One: "Lane detection" -> Where am I? */
            LANE lane = getLane(car_d);

            /* Step Two: Get SF info to create the Road -> Sensor Fusion Elements (Just cars) */
            Road road;
            for (int i = 0; i < sensor_fusion.size(); ++i) {

              // It's called vehicle instead of car just to do not get wrong with our car variables.
              auto sfv = sensor_fusion[i];
              Vehicle vehicle (sfv[0], sfv[1], sfv[2], sfv[3], sfv[4], sfv[5], sfv[6]);
              LANE vehicle_lane = getLane(vehicle.d);

              if (vehicle_lane == LEFT) {
                road.left_lane.push_back(vehicle);
              } else if (vehicle_lane == CENTER) {
                road.center_lane.push_back(vehicle);
              } else {
                road.right_lane.push_back(vehicle);
              }

            }
            /* Road Check */
            // cout << road.left_lane.size() <<  " ";
            // cout << road.center_lane.size() <<  " ";
            // cout << road.right_lane.size() <<  " ";

            /* FROM UDACITY DOCS:
            Previous path
            Previous path data given to the Planner
            //Note: Return the previous list but with processed points removed, can be a nice tool to show how far along the path has processed since last time.
            ["previous_path_x"] The previous list of x points previously given to the simulator
            ["previous_path_y"] The previous list of y points previously given to the simulator

            Previous path's end s and d values
            ["end_path_s"] The previous list's last point's frenet s value
            ["end_path_d"] The previous list's last point's frenet d value

            Using Previous Path Points
            We still use 50 points as in our last experiment, but first we add the previous path points that we used during the last cycle.
            Then we add new points in reference to the previous path until we have 50 points. Using information from the previous path can help
            ensure that there is a smooth transition from cycle to cycle but the longer the previous path the less consideration the car is taking
            for dynamic changes in its environment. Ideally maybe only a small starting portion of the previous path should be used and the rest of
            the path is then generated based on new data from the car's sensor fusion information. */

            /*****************
            ******DRIVE*******
            *****************/

            double pos_x;
            double pos_y;
            double pos_s;
            double pos_d;
            double angle;

            int path_size = previous_path_x.size();

            for(int i = 0; i < path_size; i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            // INIT
            if(path_size == 0) {
              pos_x = car_x;
              pos_y = car_y;
              pos_s = car_s;
              pos_d = car_d;
              angle = deg2rad(car_yaw);
            }
            else {
              pos_x = previous_path_x[path_size-1];
              pos_y = previous_path_y[path_size-1];
              pos_s = end_path_s;
              pos_d = end_path_d;

              double pos_x2 = previous_path_x[path_size-2];
              double pos_y2 = previous_path_y[path_size-2];
              angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
            }

            double wp_x, wp_y, wp_dx, wp_dy;
            double dist_inc = 0.4;
            for(int i = 0; i < 50-path_size; i++) {

              /* TESTING LINE */
              // pos_x += (dist_inc)*cos(angle);
              // pos_y += (dist_inc)*sin(angle);
              // next_x_vals.push_back(pos_x);
              // next_y_vals.push_back(pos_y);

              /* TESTING CIRCLE */
              // pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
              // pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
              // next_x_vals.push_back(pos_x);
              // next_y_vals.push_back(pos_y);

              /* SAME LANE */
              pos_s += dist_inc;

              // spline interpolation
              wp_x = wp_spline_x(pos_s);
              wp_y = wp_spline_y(pos_s);
              wp_dx = wp_spline_dx(pos_s);
              wp_dy = wp_spline_dy(pos_s);

              // center lane -> d = 6
              pos_x = wp_x + 6 * wp_dx;
              pos_y = wp_y + 6 * wp_dy;

              next_x_vals.push_back(pos_x);
              next_y_vals.push_back(pos_y);

            }

            // Update next point
            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;

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
