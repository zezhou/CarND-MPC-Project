#include "MPC.h"
#include <math.h>
#include <chrono>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "matplotlibcpp.h"
#include "util.h"
#include "csv.h"
//#include "gflags/gflags.h"
namespace plt = matplotlibcpp;

using CppAD::AD;

//
// Helper functions to fit and evaluate polynomials.
//

Trajectory get_test_trajectory() {
    Trajectory points;
    io::CSVReader<2> in("../lake_track_waypoints.csv");
    in.read_header(io::ignore_extra_column, "x", "y");
    Point point;
    double x; double y;
    while(in.read_row(x, y)){
        point.x = x;
        point.y = y;
        points.push_back(point);
    }
    return points;
}

std::vector<Eigen::VectorXd> get_segment_trajectory(int seq, Trajectory trajectory) {
    Point start_point = trajectory[seq];
    Point middle_point = trajectory[seq+1];
    Point middle_point2 = trajectory[seq+2];
    Point last_point = trajectory[seq + 3];
    Eigen::VectorXd ptsx(4); 
    Eigen::VectorXd ptsy(4); 
    ptsx << start_point.x, middle_point.x, middle_point2.x, last_point.x;
    ptsy << start_point.y, middle_point.y, middle_point2.y, last_point.y;
    return {ptsx, ptsy};
}

std::vector<double> x_vals = {};
std::vector<double> y_vals = {};
std::vector<double> psi_vals = {};
std::vector<double> v_vals = {};
std::vector<double> cte_vals = {};
std::vector<double> epsi_vals = {};
std::vector<double> delta_vals = {};
std::vector<double> a_vals = {};
    // The cross track error is calculated by evaluating at polynomial at x, f(x)
    // and subtracting y.
    
    // Due to the sign starting at 0, the orientation error is -f'(x).
    // derivative of coeffs[0] + coeffs[1] * x -> coeffs[1]

void record(std::vector<double> vars){
    x_vals.push_back(vars[0]);
    y_vals.push_back(vars[1]);
    psi_vals.push_back(vars[2]);
    v_vals.push_back(vars[3]);
    cte_vals.push_back(vars[4]);
    epsi_vals.push_back(vars[5]);
    delta_vals.push_back(vars[6]);
    a_vals.push_back(vars[7]);
    log(vars);
}

int main(int argc, char* argv[]) {
    //gflags::ParseCommandLineFlags(&argc, &argv, true);
    MPC mpc;
    // map's coordinate system, which is different than the car's coordinate system.
    Trajectory test_trajectory = get_test_trajectory();
    size_t iters = 20;
    int seq = 0;
    double x = test_trajectory[seq].x;
    double y = test_trajectory[seq].y;
    double psi = 0;
    double v = 10;

    Eigen::VectorXd state(6);

    for (const auto point: test_trajectory) {
        std::cout << "Seq: " << seq << std::endl;
        //std::cout << "Target point: (" << point.x << "," << point.y << ")" << std::endl;
        std::vector<Eigen::VectorXd> segment = get_segment_trajectory(seq, test_trajectory);
        Eigen::VectorXd ptsx_ = segment[0];
        Eigen::VectorXd ptsy_ = segment[1];
        vector<double> ptrx_;
        vector<double> ptry_;
        for (size_t i = 0; i < ptsx_.size(); i++) {
            Point point = world_coordinate2vehicle_coordinate({ptsx_[i], ptsy_[i]}, {x, y}, psi);
            ptrx_.push_back(point.x);
            ptry_.push_back(point.y);
        }
          Eigen::VectorXd ptrx = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptrx_.data(), ptrx_.size());
          Eigen::VectorXd ptry = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ptry_.data(), ptry_.size()); 
          

        auto coeffs = polyfit(ptrx, ptry, 3);
        //auto coeffs = polyfit(segment[0], segment[1], 2);
        double cte = polyeval(coeffs, 0);
        //double epsi = psi - atan(2* x * coeffs[2] + coeffs[1]);
        double epsi = -atan(coeffs[1]);
        state << x, y, psi, v, cte, epsi;

        record({state[0],state[1],state[2],state[3],state[4],state[5],{},{}});

        for (size_t i = 0; i < iters; i++) {
            std::cout << "Iteration " << i << std::endl;

            auto vars = mpc.Solve(state, coeffs);
            x = state[0];
            y = state[1];
            psi = state[2];
            v = state[3];

            x_vals.push_back(vars[0]);
            y_vals.push_back(vars[1]);
            psi_vals.push_back(vars[2]);
            v_vals.push_back(vars[3]);
            cte_vals.push_back(vars[4]);
            epsi_vals.push_back(vars[5]);
            delta_vals.push_back(vars[6]);
            a_vals.push_back(vars[7]);
            state << vars[0], vars[1], vars[2], vars[3], vars[4], vars[5];
            log(vars);
            double steer_value = vars[6]/deg2rad(25);
            std::cout << "steer_value: " << steer_value << std::endl;
            double throttle_value = vars[7];
            std::cout << "throttle_value: " << throttle_value << std::endl;
            if (std::abs(vars[4]) < 0.001) {
                break;
            }
        }
        /*
        std::vector<double> vars;
        vars = mpc.Solve(state, coeffs);
        state << vars[0], vars[1], vars[2], vars[3], vars[4], vars[5];
        std::cout << "Control result: " << std::endl;
        record(vars);
        x = state[0];
        y = state[1];
        psi = state[2];
        v = state[3];
        */
        seq++;


        if (seq > 2) {
            break;
        }
    }

  // Plot values
  // NOTE: feel free to play around with this.
  // It's useful for debugging!
  plt::subplot(3, 1, 1);
  plt::title("CTE");
  plt::plot(cte_vals);
  plt::subplot(3, 1, 2);
  plt::title("Delta (Radians)");
  plt::plot(delta_vals);
  plt::subplot(3, 1, 3);
  plt::title("Velocity");
  plt::plot(v_vals);

  plt::show();
}
