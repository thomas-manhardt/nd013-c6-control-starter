/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
        Aaron Brown
 **********************************************/

/**
 * @file main.cpp
 **/

#include <string>
#include <array>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <vector>
#include <iostream>
#include <fstream>
#include <typeinfo>

#include "json.hpp"
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>
#include "Eigen/QR"
#include "behavior_planner_FSM.h"
#include "motion_planner.h"
#include "planning_params.h"
#include "utils.h"
#include "pid_controller.h"

#include <limits>
#include <iostream>
#include <fstream>
#include <uWS/uWS.h>
#include <math.h>
#include <vector>
#include <cmath>
#include <time.h>

using namespace std;
using json = nlohmann::json;

#define _USE_MATH_DEFINES

string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("{");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

double angle_between_points(double x1, double y1, double x2, double y2){
  return atan2(y2-y1, x2-x1);
}

BehaviorPlannerFSM behavior_planner(
      P_LOOKAHEAD_TIME, P_LOOKAHEAD_MIN, P_LOOKAHEAD_MAX, P_SPEED_LIMIT,
      P_STOP_THRESHOLD_SPEED, P_REQ_STOPPED_TIME, P_REACTION_TIME,
      P_MAX_ACCEL, P_STOP_LINE_BUFFER);

// Decalre and initialized the Motion Planner and all its class requirements
MotionPlanner motion_planner(P_NUM_PATHS, P_GOAL_OFFSET, P_ERR_TOLERANCE);

bool have_obst = false;
vector<State> obstacles;

int main ()
{
  cout << "starting server" << endl;
  uWS::Hub h;

  double new_delta_time;
  int i = 0;

  fstream file_steer;
  file_steer.open("steer_pid_data.txt", std::ofstream::out | std::ofstream::trunc);
  file_steer.close();
  fstream file_throttle;
  file_throttle.open("throttle_pid_data.txt", std::ofstream::out | std::ofstream::trunc);
  file_throttle.close();

  time_t prev_timer;
  time_t timer;
  time(&prev_timer);

  // Steering PID (tuned to hold a bit more left when needed)
  PID pid_steer = PID();
  pid_steer.Init(0.30, 0.01, 0.4, 1.2, -1.2);

  // Throttle PID (keep your starting gains)
  PID pid_throttle = PID();
  pid_throttle.Init(0.35, 0.01, 0.2, 1.0, -1.0);

  h.onMessage([&pid_steer, &pid_throttle, &new_delta_time, &timer, &prev_timer, &i, &prev_timer](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
  {
    auto s = hasData(data);

    if (s != "") {

      auto data = json::parse(s);

      // create file to save values
      fstream file_steer;
      file_steer.open("steer_pid_data.txt");
      fstream file_throttle;
      file_throttle.open("throttle_pid_data.txt");

      vector<double> x_points = data["traj_x"];
      vector<double> y_points = data["traj_y"];
      vector<double> v_points = data["traj_v"];
      double yaw = data["yaw"];
      double velocity = data["velocity"];
      double sim_time = data["time"];
      double waypoint_x = data["waypoint_x"];
      double waypoint_y = data["waypoint_y"];
      double waypoint_t = data["waypoint_t"];
      bool is_junction = data["waypoint_j"];
      string tl_state = data["tl_state"];

      double x_position = data["location_x"];
      double y_position = data["location_y"];
      double z_position = data["location_z"];

      // Refresh obstacles every cycle (consider dynamic actors)
      obstacles.clear();
      {
        vector<double> x_obst = data["obst_x"];
        vector<double> y_obst = data["obst_y"];
        bool tmp_flag = false;
        for (int k = 0; k < (int)x_obst.size(); ++k) {
          State ob;
          ob.location.x = x_obst[k];
          ob.location.y = y_obst[k];
          obstacles.push_back(ob);
        }
      }

      State goal;
      goal.location.x = waypoint_x;
      goal.location.y = waypoint_y;
      goal.rotation.yaw = waypoint_t;

      vector< vector<double> > spirals_x;
      vector< vector<double> > spirals_y;
      vector< vector<double> > spirals_v;
      vector<int> best_spirals;

      // Generate path from planner
      {
        State ego_state;
        ego_state.location.x = x_points.back();
        ego_state.location.y = y_points.back();
        ego_state.velocity.x = velocity;
        if (x_points.size() > 1) {
          ego_state.rotation.yaw = angle_between_points(
            x_points[x_points.size()-2], y_points[y_points.size()-2],
            x_points[x_points.size()-1], y_points[y_points.size()-1]);
          ego_state.velocity.x = v_points.back();
          if (velocity < 0.01) ego_state.rotation.yaw = yaw;
        }
        Maneuver behavior = behavior_planner.get_active_maneuver();
        goal = behavior_planner.state_transition(ego_state, goal, is_junction, tl_state);

        if (behavior == STOPPED) {
          int max_points = 20;
          double px = x_points.back();
          double py = y_points.back();
          while ((int)x_points.size() < max_points) {
            x_points.push_back(px);
            y_points.push_back(py);
            v_points.push_back(0.0);
          }
        } else {
          auto goal_set = motion_planner.generate_offset_goals(goal);
          auto spirals = motion_planner.generate_spirals(ego_state, goal_set);
          auto desired_speed = utils::magnitude(goal.velocity);
          State lead_car_state;

          if (spirals.size() == 0) {
            cout << "Error: No spirals generated " << endl;
          } else {
            for (int si = 0; si < (int)spirals.size(); ++si) {
              auto trajectory = motion_planner._velocity_profile_generator.generate_trajectory(
                  spirals[si], desired_speed, ego_state, lead_car_state, behavior);
              vector<double> sx, sy, sv;
              sx.reserve(trajectory.size());
              sy.reserve(trajectory.size());
              sv.reserve(trajectory.size());
              for (int tj = 0; tj < (int)trajectory.size(); ++tj) {
                sx.push_back(trajectory[tj].path_point.x);
                sy.push_back(trajectory[tj].path_point.y);
                sv.push_back(trajectory[tj].v);
              }
              spirals_x.push_back(std::move(sx));
              spirals_y.push_back(std::move(sy));
              spirals_v.push_back(std::move(sv));
            }
            best_spirals = motion_planner.get_best_spiral_idx(spirals, obstacles, goal);

            int best_spiral_idx = -1;
            if (!best_spirals.empty()) best_spiral_idx = best_spirals.back();

            // Append next points from selected spiral
            if (best_spiral_idx >= 0) {
              int index = 0;
              int max_points = 20;
              int add_points = spirals_x[best_spiral_idx].size();
              while ((int)x_points.size() < max_points && index < add_points) {
                x_points.push_back(spirals_x[best_spiral_idx][index]);
                y_points.push_back(spirals_y[best_spiral_idx][index]);
                v_points.push_back(spirals_v[best_spiral_idx][index]);
                index++;
              }
            }
          }
        }
      }

      // Save time and compute delta time
      time(&timer);
      new_delta_time = difftime(timer, prev_timer);
      prev_timer = timer;

      ////////////////////////////////////////
      // Steering control (with small left bias if blocker ahead in-lane)
      ////////////////////////////////////////

      pid_steer.UpdateDeltaTime(new_delta_time);

      double vect_direction_x = x_points.back() - x_points[0];
      double vect_direction_y = y_points.back() - y_points[0];
      double trajectory_heading = atan2(vect_direction_y, vect_direction_x);
      double error_steer = trajectory_heading - yaw;

      // Small left bias only if in-lane blocker detected
      {
        const double ch = std::cos(trajectory_heading);
        const double sh = std::sin(trajectory_heading);
        const double lat_lane = 2.5; // in-lane width
        const double look = 25.0;    // meters
        bool inlane_blocker = false;
        for (const auto& ob : obstacles) {
          double dx = ob.location.x - x_points.back();
          double dy = ob.location.y - y_points.back();
          double xf =  dx * ch + dy * sh;
          double yl = -dx * sh + dy * ch;
          if (xf > 0.0 && xf < look && std::fabs(yl) < lat_lane) {
            inlane_blocker = true; break;
          }
        }
        if (inlane_blocker) {
          error_steer += 0.05; // ~3 degrees left
        }
      }

      pid_steer.UpdateError(error_steer);
      double steer_output = pid_steer.TotalError();

      // Save steer data
      file_steer.seekg(std::ios::beg);
      for(int j=0; j < i - 1; ++j) {
          file_steer.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      }
      file_steer  << i << " " << error_steer << " " << steer_output << endl;

      ////////////////////////////////////////
      // Throttle control (shape with steering to drift left safely)
      ////////////////////////////////////////

      pid_throttle.UpdateDeltaTime(new_delta_time);

      double v_obj = v_points.empty() ? 0.0 : v_points.back();
      double error_throttle = v_obj - velocity;

      pid_throttle.UpdateError(error_throttle);
      double u = pid_throttle.TotalError();

      // Ensure positive error -> positive throttle
      if ((error_throttle > 0 && u < 0) || (error_throttle < 0 && u > 0)) {
        u = -u;
      }

      double throttle_output = 0.0;
      double brake_output = 0.0;
      const double deadband = 0.02;

      if (u > deadband) {
        throttle_output = std::min(u, 1.0);
      } else if (u < -deadband) {
        brake_output = std::min(-u, 1.0);
      }

      // Throttle shaping vs steering: slow down when steering is large
      {
        double steer_abs = std::fabs(steer_output);
        double throttle_scale = 1.0 - std::min(0.8, 0.8 * steer_abs); // up to -80%
        throttle_output *= std::max(0.2, throttle_scale);             // keep some crawl
      }

      // Save throttle data
      file_throttle.seekg(std::ios::beg);
      for(int j=0; j < i - 1; ++j){
          file_throttle.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
      }
      file_throttle  << i << " " << error_throttle << " " << brake_output << " " << throttle_output << endl;

      // Send control
      json msgJson;
      msgJson["brake"] = brake_output;
      msgJson["throttle"] = throttle_output;
      msgJson["steer"] = steer_output;

      msgJson["trajectory_x"] = x_points;
      msgJson["trajectory_y"] = y_points;
      msgJson["trajectory_v"] = v_points;
      msgJson["spirals_x"] = spirals_x;
      msgJson["spirals_y"] = spirals_y;
      msgJson["spirals_v"] = spirals_v;
      msgJson["spiral_idx"] = best_spirals;
      msgJson["active_maneuver"] = behavior_planner.get_active_maneuver();

      // for high update rate use 19 for slow update rate use 4
      msgJson["update_point_thresh"] = 16;

      auto msg = msgJson.dump();

      i = i + 1;
      file_steer.close();
      file_throttle.close();

      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    }
  });

  h.onConnection([](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
  {
    cout << "Connected!!!" << endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length)
  {
    ws.close();
    cout << "Disconnected" << endl;
  });

  int port = 4567;
  if (h.listen("0.0.0.0", port))
  {
    cout << "Listening to port " << port << endl;
    h.run();
  }
  else
  {
    cerr << "Failed to listen to port" << endl;
    return -1;
  }
}

