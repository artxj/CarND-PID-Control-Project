#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

// re-inits the robot PID with given params, resets the steps count and error
void resetRobot(PID &pid, double *params, unsigned int &steps_count, double &err) {
  pid.Init(params[0], params[1], params[2]);
  steps_count = 0;
  err = 0;
}

int main() {
  uWS::Hub h;

  PID steerPID;
  PID speedPID;

  double params[] = { 0.0, 0.0, 0.0 };
  double dp[] = { 1.0, 1.0, 1.0 };

  bool useSpeedPID = false; // set to true to use both PIDs

  bool isSteerTwiddle = false;
  if (isSteerTwiddle) {
    steerPID.Init(params[0], params[1], params[2]);
  } else {
    // uncomment next params if only steer PID is used without the speed one
    if (useSpeedPID) {
      steerPID.Init(0.6, 0, 5.3);
    } else {
      steerPID.Init(0.5, 0, 5.70568);
    }
  }
  bool isSpeedTwiddle = false;
  if (isSpeedTwiddle) {
    speedPID.Init(params[0], params[1], params[2]);
  } else {
    speedPID.Init(0.85, 0.0, 3.1);
  }

  unsigned int current_step = 0;
  const unsigned int max_steps = 10000;
  const double twiddle_threshold = 0.2;
  bool plus_dp_run = true; // indicates whether we're in the first run cycle (params += dp)
  double best_err = 0.0;
  bool best_err_init = false;
  double err = 0.0;
  unsigned int current_i = 0;

  h.onMessage([&steerPID, &speedPID, &current_step, &max_steps, &isSteerTwiddle, &isSpeedTwiddle,
    &params, &dp, &twiddle_threshold, &plus_dp_run, &best_err, &best_err_init, &err, &current_i,
    &useSpeedPID]
    (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());

          // Twiddle code - used when tuning the PID params
          if (isSteerTwiddle || isSpeedTwiddle) {
            if (best_err_init && current_i == 0 && dp[0] + dp[1] + dp[2] < twiddle_threshold) {
              // done with twiddling
              std::cout << "Twiddle params found: " << std::endl;
              std::cout << params[0] << ", " << params[1] << ", " << params[2] << std::endl;
              exit;
            }

            if (err > 500) current_step = max_steps;

            // twiddle mode on - update the error
            if (current_step > 100) err += cte * cte;

            if (current_step == max_steps) {
              // we've reached the last step in current run
              if (!best_err_init) {
                // first run, need to save best error
                best_err = err;
                best_err_init = true;
                params[current_i] += dp[current_i];
              } else {
                std::cout << "Current error is " << err << std::endl;
                std::cout << "Params are " << params[0] << ", " << params[1] << ", " << params[2] << std::endl;
                if (err < best_err) {
                  // new error is better
                  dp[current_i] *= 1.1;
                  plus_dp_run = true;
                  current_i = (current_i + 1) % 3;
                  params[current_i] += dp[current_i];
                } else {
                  // check if only first run is over
                  if (plus_dp_run) {
                    plus_dp_run = false;
                    params[current_i] -= 2 * dp[current_i];
                  } else {
                    params[current_i] += dp[current_i];
                    dp[current_i] *= 0.9;
                    plus_dp_run = true;
                    current_i = (current_i + 1) % 3;
                    params[current_i] += dp[current_i];
                  }
                }
              }
              std::cout << "Restarting the run cycle" << std::endl;
              // run cycle is over, let's reset all for new run
              std::string msg = "42[\"reset\",{}]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              if (isSteerTwiddle) {
                resetRobot(steerPID, params, current_step, err);
              } else if (isSpeedTwiddle) {
                resetRobot(speedPID, params, current_step, err);
              }
              return;
            } else {
              // inc steps
              // std::cout << "Step # " << current_step << " of run with param index " << current_i << std::endl;
              current_step += 1;
            }
          }

          /*
          * Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          steerPID.UpdateError(cte);
          double steer_value = steerPID.TotalError();
          if (steer_value < -1.0) steer_value = -1.0;
          if (steer_value > 1.0) steer_value = 1.0;

          double throttle = 0.3;
          if (useSpeedPID) {
            speedPID.UpdateError(fabs(cte));
            throttle = 1.0 + speedPID.TotalError();
          }
          if (isSpeedTwiddle && throttle < 0 && speed < 1.0) {
            // to prevent issues when car is moving backwards
            err = 50000;
          }

          // DEBUG
          if (!isSteerTwiddle && !isSpeedTwiddle) {
            std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          if (!isSteerTwiddle && !isSpeedTwiddle) {
            std::cout << msg << std::endl;
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

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
