#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <math.h>

#include "json.hpp"

#include "spline.h"
#include "vehicle.h"
#include "road.h"
#include "planner.h"

using namespace std;
using namespace tk; // spline

// for convenience
using json = nlohmann::json;

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


  // The max s value before wrapping around the track back to 0
  int event_counter = 0;
  double max_s = 6945.554;

  // Road & Vehicle & Planner instances
  Road road;
  Vehicle car;
  Planner planner;

  h.onMessage([
    &road,
    &car,
    &planner,
    &wp_spline_x,
    &wp_spline_y,
    &wp_spline_dx,
    &wp_spline_dy,
    &event_counter](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          event_counter++;
          cout << "EVENT: " << event_counter << endl;

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

          /*****************
          ******CONTEXT*****
          *****************/

          cout << "-------------CAR--------------" << endl;
          cout << "Car S: " << car_s << " " << "Car speed: " << car_speed << endl;

          car.update_vehicle_state(car_x, car_y, car_speed, car_s, car_d, car_yaw);

          /* Step One: "Lane detection" -> Where am I? */
          LANE lane = car.lane();
          // cout << "Car Lane: " << lane << endl;

          vector<Vehicle> left_lane;
          vector<Vehicle> center_lane;
          vector<Vehicle> right_lane;

          /* Step Two: Get SF info to create the Road -> Sensor Fusion Elements (Just cars) */
          for (int i = 0; i < sensor_fusion.size(); ++i) {
            // It's called vehicle instead of car just to do not get wrong with our car variables.
            int id = sensor_fusion[i][0];
            double x = sensor_fusion[i][1];
            double y = sensor_fusion[i][2];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double s = sensor_fusion[i][5];
            double d = sensor_fusion[i][6];
            double v = sqrt(vx*vx + vy*vy);

            Vehicle vehicle (id, x, y, v, s, d);
            LANE vehicle_lane = vehicle.lane();

            if (vehicle_lane == LANE::LEFT) {
              left_lane.push_back(vehicle);
            } else if (vehicle_lane == LANE::CENTER) {
              center_lane.push_back(vehicle);
            } else {
              right_lane.push_back(vehicle);
            }
          }
          // Update road
          road.update_road(left_lane, center_lane, right_lane);

          /* Road Check */
          //cout << "---------------ROAD---------------" << endl;
          //cout << "Left: " << road.get_lane(LEFT).size() << " " << "Center: " << road.get_lane(CENTER).size() << " " << "Right: " << road.get_lane(CENTER).size() << endl;

          /*****************
          ******CORE*****
          *****************/

          // Previous path
          int n = previous_path_x.size();
          cout << n << endl;
          for(int i = 0; i < n; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          if (car.is_stopped()) {
            // block
            car.lock_start();

            n = 50*4.5; // 0 to max speed? 50*5?
            double T = n * AT;
            double target_s = car.get_s() + 40.0;
            double target_v = SPEED_LIMIT;

            vector<double> start = {car.get_s(), car.get_v(), 0.0};
            vector<double> end = {target_s, target_v, 0.0};
            vector<double> poly = planner.JMT(start, end, T);

            car.set_previous_s(end);

            double t, next_s, mod_s, wp_x, wp_y, wp_dx, wp_dy, next_x, next_y;
            for(int i = 0; i < n; i++) {

              /* JMT */
              cout << "----------JMT----------" << endl;
              t = AT*i;
              cout << "t= " << t << endl;
              next_s = 0.0;
              for (int a = 0; a < poly.size(); ++a) {
                next_s += poly[a] * pow(t, a);
              }

              mod_s = fmod(next_s, TRACK_DISTANCE);

              //@TODO
              // spline interpolation
              wp_x = wp_spline_x(mod_s);
              wp_y = wp_spline_y(mod_s);
              wp_dx = wp_spline_dx(mod_s);
              wp_dy = wp_spline_dy(mod_s);

              next_x = roundf((wp_x + wp_dx * 6.0)*100) / 100;
              next_y = roundf((wp_y + wp_dy * 6.0)*100) / 100;

              next_x_vals.push_back(next_x);
              next_y_vals.push_back(next_y);
            }
          }

          // New path
          if (n < MIN_PATH_POINTS) {

            /*****************
            ******DRIVE*******
            *****************/

            //@TODO ACCELERATION (vNow-vPrevios/AT*NUMPOINTSCONSUMED)

            // get target states based on behavior s component
            cout << "a" << endl;
            double target_s = car.prev_s()[0] + AT*CYCLES*POINTS * car.prev_s()[1];
            double target_v = car.prev_s()[1];
            cout << "a" << endl;

            vector<double> start = {car.prev_s()[0], car.prev_s()[1], 0.0};
            vector<double> end = {target_s, target_v, 0.0};
            vector<double> poly = planner.JMT(start, end, AT*CYCLES*POINTS);
            cout << "a" << endl;

            car.set_previous_s(end);
            cout << "a" << endl;

            double t, next_s, mod_s, wp_x, wp_y, wp_dx, wp_dy, next_x, next_y;
            for(int i = 0; i < CYCLES*POINTS; i++) {

              /* JMT */
              cout << "----------JMT----------" << endl;
              t = AT*i;
              cout << "t= " << t << endl;
              next_s = 0.0;
              for (int a = 0; a < poly.size(); ++a) {
                next_s += poly[a] * pow(t, a);
              }

              mod_s = fmod(next_s, TRACK_DISTANCE);

              //@TODO
              // spline interpolation
              wp_x = wp_spline_x(mod_s);
              wp_y = wp_spline_y(mod_s);
              wp_dx = wp_spline_dx(mod_s);
              wp_dy = wp_spline_dy(mod_s);

              next_x = roundf((wp_x + wp_dx * 6.0)*100) / 100;
              next_y = roundf((wp_y + wp_dy * 6.0)*100) / 100;

              next_x_vals.push_back(next_x);
              next_y_vals.push_back(next_y);
            }
          }

          // Update next point
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          cout << "\n" << endl;

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
