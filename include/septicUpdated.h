#pragma once

#include <Eigen/Dense>
#include <Eigen/LU>
#include <iostream>
#include <vector>

class SEPTIC
{
public:
     SEPTIC(int dof);

     void calcCoeffs(std::vector<double> init_pos, std::vector<double> final_pos,
                     double maxvel, double maxacc, double maxjerk, bool degrees,
                     std::vector<double> init_vel = {},
                     std::vector<double> final_vel = {},
                     std::vector<double> init_accel = {},
                     std::vector<double> final_accel = {},
                     std::vector<double> init_jerk = {},
                     std::vector<double> final_jerk = {});

     bool generatePathAndVel(double t, std::vector<double> &position,
                             std::vector<double> &velocity,
                             std::vector<double> &acceleration,
                             std::vector<double> &jerk);

     void printVec(std::vector<double> input);
     void printMat(std::vector<std::vector<double>> input);

     std::vector<std::vector<double>> &getPath() { return std::ref(_finalPath); }
     std::vector<std::vector<double>> &getVel() { return std::ref(_finalVel); }
     std::vector<std::vector<double>> &getAccel() { return std::ref(_finalAccel); }

     ~SEPTIC();

private:
     int _dof;
     // int _waypts;
     double _maxvel;
     double _finalTime;
     bool callCount = false;

     std::vector<std::vector<double>> _finalPath;
     std::vector<std::vector<double>> _finalVel;
     std::vector<std::vector<double>> _finalAccel;
     std::vector<std::vector<double>> _finalJerk;
     std::vector<std::vector<double>> _finalConstMat;

     Eigen::VectorXd _timeStep;
};