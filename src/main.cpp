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

// Fit a polynomial. dapted from http://bit.ly/2hAf75b
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
          double psi_u = j[1]["psi_unity"];
          double v = j[1]["speed"];
          double delta = j[1]["steering_angle"]; // this isn't received it's sent
          double accel = j[1]["throttle"];

          // transform waypoint coords from global to vehicle
          Eigen::VectorXd ptsx_trans(ptsx.size());
          Eigen::VectorXd ptsy_trans(ptsy.size());
          for (int i = 0; i < ptsx.size(); i++) {
            ptsx_trans(i) = (ptsx[i]-px)*cos(0-psi) - (ptsy[i]-py)*sin(0-psi);
            ptsy_trans(i) = (ptsy[i]-py)*cos(0-psi) + (ptsx[i]-px)*sin(0-psi);
          }

          printf("\n[BEFORE] [px] %+6.2f [py] %+6.2f [psi] %+6.3f "
            "[psi_u] %+6.2f [delta] %+6.5f "
            "[v] %+6.3f [accel] %+6.2f\n", px, py, psi, psi_u, delta, v, accel);

          double Lf = 2.67; // taken from MPC.CppAD
          v *= 0.44704; // to go from miles/hour to meters/sec
          v += accel * LATENCY_SEC;
          psi = - v * delta / Lf * LATENCY_SEC;
          px = v * LATENCY_SEC;

          printf("[AFTER]  [px] %+6.2f [py] %+6.2f [psi] %+6.3f "
            "[psi_u] %+6.2f [delta] %+6.5f "
            "[v m/s] %+6.3f [accel] %+6.2f\n", px, py, psi, psi_u, delta, v, accel);

          // fit a third order polynomial to the 6 given waypoints
          Eigen::VectorXd coeffs = polyfit(ptsx_trans, ptsy_trans, 3);
          // compute the cross track errors
          double cte = polyeval(coeffs, px);
          // compute orientation error
          double despsi = atan(3*coeffs[3]*px*px + 2*coeffs[2]*px + coeffs[1]);
          double epsi = despsi - psi;
          // double epsi = -atan(coeffs[1]);
          printf("[cte] %+6.2f [despi] %+6.2f [epsi] %+6.2f\n", cte, despsi, epsi);
          // ready the state vars for mpc.Solve
          Eigen::VectorXd state(6);

          state << px,0,psi, v, cte, epsi;

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

          // log output
          std::cout << "[steer] " << steer_value
                    << " [steer deg] " << vars[0]
                    << " [throt] " << throttle_value
                    << " [cte] " << cte << " [epsi] " << epsi
                    << " [delta] " << delta
                    << "[speed] "<< v << std::endl;

          // std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));

          // restart
          if (abs(cte) > 100){
            std::string msg = "42[\"reset\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }

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
