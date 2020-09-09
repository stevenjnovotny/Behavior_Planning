#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

bool laneIsClear(double car_s, int car_d, bool wantToPass, int lane, int plan_size, vector<vector<double>> sensor_fusion) {
  
    //std::cout << car[0] << " -  v:" << v << "  gap:" << abs(gap) << "  leading:" << ahead << std::endl; 

  return false;
}

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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	std::istringstream iss(line);
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

  // Starting conditions
  int lane = 1;
  double ref_vel = 0.0;
  int current_wp = 0;
  int last_pass_wp = 0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &current_wp, &ref_vel, &lane, &last_pass_wp]
    (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {

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
          
          // create Json msg object
          json msgJson;

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

          // Sensor Fusion Data, a list of all other cars on the same side 
          //     of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];


          // project constraints and constants
          const double max_speed = 49.5;
          const double mph2ms = 0.44704;
          const double ms2mph = 1.0/mph2ms;
          // (v2-v1)/dt = a
          const double max_acc = 10.0 * 0.02;

          int prev_size = previous_path_x.size();
          if (prev_size > 0) {
            car_s = end_path_s;
          }


          /**
          * Fintite state machine [keep lane; prep for lane change; lane change]
          * evaluate and choose state
          * - look at lane ahead
          * -- evaluate speed vs max speed
          * -- is lane clear? does it allow max speed
          * -- if not...  (-----> prepare to change lanes)
          * - look at lanes to left and right
          * -- evaluate speed of nearby cars
          * -- is either lane clear (-----> change lanes)
          * - change lanes
          * -- pick desired lane
          * -- generate trajectory for lane shift
          */




          // are neighboring lanes blocked? [left,same, right]
          vector<bool> lanes_blocked = {false, false, false};
          double obstacle_dist = 1000.0;
          vector<double> lane_speeds = {1000, 1000, 1000};
          vector<double> forward_gaps = {1000, 1000, 1000};
          
          for ( int i = 0; i < sensor_fusion.size(); i++ ) {
            // values for car i
            float d = sensor_fusion[i][6];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy);
            double check_car_s = sensor_fusion[i][5];
            check_car_s += ((double)prev_size*0.02*check_speed);

            int car_lane = -1;
            if ( d > 0.0 && d < 4.0 ) {
              car_lane = 0;
            } else if ( d > 4.0 && d < 8.0 ) {
              car_lane = 1;
            } else if ( d > 8.0 && d < 12.0 ) {
              car_lane = 2;
            } else {
              // we don't care...
              continue;
            }

            int rel_lane = -1;
            double gap = car_s - check_car_s;

            if ( car_lane == lane ) {
              lanes_blocked[1] = lanes_blocked[1] || (check_car_s > car_s && check_car_s - car_s < 30);
              if (gap < obstacle_dist && gap < 0.0) { obstacle_dist = gap;}
            } 
            else if ( car_lane - lane == -1 ) {
              rel_lane = 0;
            } else if ( car_lane - lane == 1 ) {
              rel_lane = 2;
            }
          
            if (rel_lane == 0 || rel_lane == 2) {
              lanes_blocked[rel_lane] = lanes_blocked[rel_lane] || (gap < 20.0  &&  gap > -20.0);
            }

            if (gap < 0.0 && abs(gap) < forward_gaps[lane]) {
              forward_gaps[car_lane] = abs(gap);
              lane_speeds[car_lane] = check_speed * ms2mph;
            } 

          }

          

          // Behavior Planning - stay (and possibly slow), change left, change right
          // Output is a target lane and a delta_v
          if (last_pass_wp > current_wp) {last_pass_wp = 0;}
          double delta_v = 0;
          if ( lanes_blocked[1] ) { 
            if ( !lanes_blocked[0] && lane > 0 && lane_speeds[lane-1] > ref_vel && current_wp > (last_pass_wp + 2)) {
              // change lanes left
              std::cout << lane << " -> ";
              lane--; 
              last_pass_wp = current_wp;
              std::cout << lane << std::endl;
            } else if ( !lanes_blocked[2] && lane < 2 && lane_speeds[lane+1] > ref_vel && current_wp > (last_pass_wp + 2)){
              // change lanes right
              std::cout << lane << " -> ";
              lane++; 
              last_pass_wp = current_wp;
              std::cout << lane << std::endl;
            } else if (ref_vel > lane_speeds[lane]) {
              // slow down
              delta_v -= max_acc;
            }
          } else {
            if ( ref_vel < max_speed - max_acc ) {
              delta_v += max_acc;
            }
          }

          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);


          // if no previous plan, set previous pts
          if ( prev_size < 2 ) {
            // project backward based on yaw
            double prev_car_x = car_x - 1.0 * cos(car_yaw);
            double prev_car_y = car_y - 1.0 * sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } else {
            // ...otherwise use points from previous plan
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          int pt_space = 30.0;
          
          for (int i=0; i<3; i++){
            vector<double> wp = getXY(car_s+(i+1)*pt_space, (2.0+4.0*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            ptsx.push_back(wp[0]);
            ptsy.push_back(wp[1]);
          }

          // Making coordinates to local car coordinates.
          for ( int i = 0; i < ptsx.size(); i++ ) {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }

          // Create the spline.
          tk::spline s;
          s.set_points(ptsx, ptsy);

          // Output path points from previous path for continuity.
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          for ( int i = 0; i < prev_size; i++ ) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate distance y position on 30 m ahead.
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);

          double x_add_on = 0;

          for( int i = 1; i < 50 - prev_size; i++ ) {
            ref_vel += delta_v;
            if ( ref_vel > max_speed ) {
              ref_vel = max_speed;
            }

            double N = target_dist/(0.02*ref_vel/2.24);
            double x_point = x_add_on + target_x/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          // provide output for troubleshooting
          int next_wp = NextWaypoint(car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y);
          if (next_wp != current_wp) {
            std::cout << next_wp << "-   s: " << map_waypoints_s[next_wp];
            std::cout << " :  left blocked: " << lanes_blocked[0] << "   right blocked: " << lanes_blocked[2] << "  target lane: " << lane << std::endl;
            if (lanes_blocked[1]) {
              std::cout << "slow car close ahead - new speed: " <<  ref_vel << "  projected dist to obs:" << abs(obstacle_dist) << std::endl;
            }
            std::cout << "Lane Speeds [0,1,2] : "; 
            for (auto s : lane_speeds) {
              std::cout << s << " ";
            }
            std::cout << " " << std::endl;
            current_wp = next_wp;
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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