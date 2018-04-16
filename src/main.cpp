#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double mph2mps(double x) {return x*0.44704;}

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

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
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

/*
 * Testing Transformations
 */
//  double theta = -M_PI/3;
//  double px = 1;
//  double py =1;
//  Eigen::Matrix3d T_c2g;
//  Eigen::Vector3d P;
//  T_c2g << cos(theta), -sin(theta), px,
//           sin(theta), cos(theta), py,
//           0, 0, 1;
//  P << 3,3,1;
//  std::cout << P << std::endl;
//  std::cout << T_c2g << std::endl;
//  Eigen::Matrix3d T_g2c;
////  T_g2c << cos(theta), sin(theta), -px,
////           -sin(theta), cos(theta), -py,
////           0, 0, 1;
//  T_g2c = T_c2g.inverse();
//  std::cout << T_g2c << std::endl;
//  Eigen::Vector3d Pc = T_g2c*P;
//  std::cout << Pc << std::endl;
//  Eigen::Vector3d P2;
//  P2 << 0,0,1;
//  theta = -theta;
//  std::cout << P2 << std::endl;


  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          const double px = j[1]["x"];
          const double py = j[1]["y"];
          const double psi = j[1]["psi"];
          const double v_mph = j[1]["speed"];

          // convert v from mph to m/s
          //const double v = v_mph;
          const double v = mph2mps(v_mph);

//          Eigen::Matrix3d T_c2g;
//          Eigen::Vector3d P;
//          for(unsigned i = 0; i < ptsx.size();++i){
//            T_c2g << cos(psi), -sin(psi), px,
//                     sin(psi), cos(psi), py,
//                     0, 0, 1;
//            P << ptsx[i], ptsy[i], 1;
//
//            // Transfor to car frame
//            const Eigen::Vector3d P_car = T_c2g.inverse()*P;
//            ptsx[i] = P_car[0];
//            ptsy[i] = P_car[1];
//          }

          // Shift the reference of the points to the car position and orientation
          for(unsigned i = 0; i < ptsx.size(); ++i){
            // shift position of the points to car origin
            const double shift_x = ptsx[i]-px;
            const double shift_y = ptsy[i]-py;

            ptsx[i] = shift_x*cos(-psi) - shift_y*sin(-psi);
            ptsy[i] = shift_x*sin(-psi) + shift_y*cos(-psi);
          }


          double* pX = &ptsx[0];
          Eigen::Map<Eigen::VectorXd> ptsx_eigen(pX,ptsx.size());

          double* pY = &ptsy[0];
          Eigen::Map<Eigen::VectorXd> ptsy_eigen(pY, ptsy.size());

          Eigen::VectorXd coeffs = polyfit(ptsx_eigen, ptsy_eigen, 3);

          /*
           * Calculate cte
           */
          // shortest distance of the car point to the polynomial
          // approximated by evaluating the polynomial for x = 0
          // frame relative to the car cte = f(px) - py reduces to f(0)
          double cte = polyeval(coeffs,0);

          /*
           * calculate epsi
           */
          // epsi = psi - psi_des, where psi_des = arctan(f'(x))
          // for f(x) = a0 + a1*x + a2*x^2 + a3*x^3, f'(x) = a1+2*a2*x + 3*a3*x^2;
          // f'(x) at (0,0) is a1
          // epsi = psi - atan[a1] at (0,0) with psi = 0
          // epsi = -atan([a1])
          double epsi = -atan(coeffs[1]);

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          double steer_value = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];

          /*
           * Account for latency by using the kinematic model to predict the state of the vehicle after latency period
           */
           const double latency = 0.2; // latency = 200ms
           const double Lf = 2.67;
           const double px_latent = v*latency;
           const double py_latent = 0;
           const double psi_latent = v*(-steer_value/Lf)*latency;
           const double v_latent = v + throttle_value*latency;
           const double cte_latent = cte + v*sin(epsi)*latency;
           const double epsi_latent = epsi + psi_latent;

          //std::cout << "velocity: " << v << "steering_angle: " << steer_value << std::endl;
          Eigen::VectorXd state(6);
          //state << 0.0, 0.0, 0.0, v, cte, epsi;
          state << px_latent, py_latent, psi_latent, v_latent, cte_latent, epsi_latent;

          auto vars = mpc.Solve(state, coeffs);
          steer_value = vars[0];
          throttle_value = vars[1];


          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value/deg2rad(25);
          // Set the acceleration limits to somethng reasonable 0-60mph in 6s ia about 5m/s^2
          // set the deceleration limit to 60-0 in about 6s (needs to stop quicker?) 5m/s^2
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          for (unsigned i = 2;i < vars.size(); ++i){
             if(i%2 == 0)
               mpc_x_vals.push_back(vars[i]);
             else
               mpc_y_vals.push_back(vars[i]);
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          double delta_x = 2.5;
          unsigned nPoints = 10;
          for(unsigned i=1; i < nPoints; ++i){
            const double x = delta_x*i;
            next_x_vals.push_back(x);
            next_y_vals.push_back(polyeval(coeffs,x));
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
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
