#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <stdio.h>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;
using Eigen::VectorXd;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
const int MAXSTEER_DEG = 25;
const double LATENCY_SEC = 0.1; // in seconds
const double Lf = 2.67;

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

// Fit a polynomial. adapted from http://bit.ly/2hAf75b
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
    // cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          // six waypoints are given at a time
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double delta = j[1]["steering_angle"];
          double accel = j[1]["throttle"];

          // transform waypoint coords from global to vehicle
          Eigen::VectorXd ptsx_trans(ptsx.size());
          Eigen::VectorXd ptsy_trans(ptsy.size());
          for (int i = 0; i < ptsx.size(); i++) {
            ptsx_trans(i) = (ptsx[i]-px)*cos(0-psi) - (ptsy[i]-py)*sin(0-psi);
            ptsy_trans(i) = (ptsy[i]-py)*cos(0-psi) + (ptsx[i]-px)*sin(0-psi);
          }

          // fit a third order polynomial to the 6 given waypoints
          Eigen::VectorXd coeffs = polyfit(ptsx_trans, ptsy_trans, 3);

          /*
          compute errors of the model
          the following assumes that px=py=0 since the model's position and yaw
          is 'tared'
           */
          double cte = polyeval(coeffs, 0);
          double epsi = -atan(coeffs[1]);

          // compute model predictions for the sake of latency
          v *= 0.44704; // convert from miles/hour to meters/sec
          px = v * LATENCY_SEC;
          py = 0; // assume zero of very little lateral movement
          psi = -v * delta * LATENCY_SEC / Lf; // `-` b/c str cmd is flipped in Unity
          epsi += epsi + psi;
          cte += v * sin(epsi) * LATENCY_SEC;
          v += accel * LATENCY_SEC;

          Eigen::VectorXd state(6);
          state << px, py , psi, v, cte, epsi;

          // compute the ideal driving commands using IPOPT
          auto vars = mpc.Solve(state, coeffs);

          // Calculate steering angle and throttle using MPC.
          // Both are between [-1, 1].
          double steer_value;
          double throttle_value;
          // NOTE: Remember to divide by deg2rad(25) before
          // you send the steering value back.
          steer_value = vars[0] / deg2rad(MAXSTEER_DEG);
          throttle_value = vars[1];

          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          for (int i = 3; i < vars.size(); i++){
            if ( i % 2 == 0 ){
              mpc_x_vals.push_back(vars[i]);
            }
            else {
              mpc_y_vals.push_back(vars[i]);
            }
          }

          // Display the waypoints/reference line
          // the points in the simulator are connected by a Yellow line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          double poly_inc = 2.5;
          int num_points = 25;
          for (int i = 0; i < num_points; i++) {
            next_x_vals.push_back( poly_inc * i );
            next_y_vals.push_back( polyeval(coeffs, poly_inc * i) );
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";

          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does not actuate the commands instantly.
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
  // program doesn't compile :-(
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
