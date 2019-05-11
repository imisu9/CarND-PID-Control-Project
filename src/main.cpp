#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include <numeric>
#include <vector>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;
  
  /**
   * Twiddle variable definition & initialization
   */
  enum Index {P=0, I, D};
  int idx = P;
  double rate = 0.1;
  vector<double> K{0.2, 0.004, 3.0};
  vector<double> dK{K[P]*rate, K[I]*rate, K[D]*rate};
  enum Trial {FIRST=1, SECOND};
  int curr_trial = FIRST;
  double sum_dK = 0.0;
  double tolerance = 0.02;  // initialized as shown in the class
  double best_error = std::numeric_limits<double>::max();
  double curr_error = 0.0;
  int MIN_ITERATION = 200;
  int iteration = 0;
  
  /**
   * TODO: Initialize the pid variable.
   */
  pid.Init(K[P], K[I], K[D]);

  h.onMessage([&pid, &dK, &K, &idx, &curr_trial, &sum_dK, &tolerance, &best_error, &curr_error, &MIN_ITERATION, &iteration]
                    (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          
          pid.UpdateError(cte);
          steer_value = -pid.TotalError();
          /**
          std::cout << "Steering Value before bounding: " << steer_value 
                    << std::endl;
          */     
          if (steer_value > 1) {
            steer_value = 1;
          } else if (steer_value < -1) {
            steer_value = -1;
          }
          
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
                    << std::endl;
          
          // Twiddle
          sum_dK = dK[P]+dK[I]+dK[D];
          std::cout << "&&&&& sum_dK: " << sum_dK << " #####" << std::endl;
          std::cout << "      dKp: " << dK[P] << " dKi: " << dK[I] << " dKd: " << dK[D] << std::endl;
          if (sum_dK > tolerance) {
            if (iteration == 0) {
              K[idx] += dK[idx];
              pid.Init(K[P], K[I], K[D]);
              curr_error += pow(cte, 2.0);
              iteration += 1;
            } else if (iteration < MIN_ITERATION) {
              curr_error += pow(cte, 2.0);
              iteration += 1;
            } else {
              curr_error /= MIN_ITERATION;
              if (curr_trial == FIRST) {
                if (curr_error < best_error) {
                  best_error = curr_error;
                  dK[idx] *= 1.1;
                  idx += 1;
                  idx %= 3;
                  
                  std::cout << "##### Update K[" << idx << "] #####" << std::endl;
                  std::cout << "      it succeeded at the FIRST trial." << std::endl;
                  std::cout << "      Kp: " << K[P] << " Ki: " << K[I] << " Kd: " << K[D] << std::endl;
                  std::cout << "      best_error: " << best_error << std::endl;
                } else {
                  curr_trial = SECOND;
                  K[idx] -= 2*dK[idx];
                  pid.Init(K[P], K[I], K[D]);
                }
              } else if (curr_trial == SECOND) {
                if (curr_error < best_error) {
                  best_error = curr_error;
                  dK[idx] *= 1.1;
                  idx += 1;
                  idx %= 3;
                  
                  std::cout << "##### Update K[" << idx << "] #####" << std::endl;
                  std::cout << "      it succeeded at the SECOND trial." << std::endl;
                  std::cout << "      Kp: " << K[P] << " Ki: " << K[I] << " Kd: " << K[D] << std::endl;
                  std::cout << "      best_error: " << best_error << std::endl;
                } else {
                  K[idx] += dK[idx];
                  pid.Init(K[P], K[I], K[D]);
                  dK[idx] *= 0.9;
                  idx += 1;
                  idx %= 3;
                  
                  std::cout << "##### Update K[" << idx << "] #####" << std::endl;
                  std::cout << "      it failed at the SECOND trial." << std::endl;
                  std::cout << "      Kp: " << K[P] << " Ki: " << K[I] << " Kd: " << K[D] << std::endl;
                  std::cout << "      best_error: " << best_error << std::endl;
                }
                curr_trial = FIRST;
              } else {
                // reset to FIRST
                curr_trial = FIRST;
              }
              // reset to iteration for MIN_ITERATION
              curr_error = 0;
              iteration = 0;
              std::string reset_msg = "42[\"reset\",{}]";
              ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
              return;
            }
          } else {
            std::cout << "We have reached to optimal K values."
                      << "Kp: " << K[P]
                      << "Ki: " << K[I]
                      << "Kd: " << K[D] << std::endl;
          }
          
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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
