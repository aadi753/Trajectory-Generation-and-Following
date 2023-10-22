#pragma once


#include <iostream>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <vector>
#include <trajectories.h>
#include <bits/stdc++.h>

class quint
{
public:
     /**
      * @brief Construct a new quint:: quint object
      *
      * @param dof
      * @param finalTime
      * @param waypoints
      */
     quint(int dof, int waypoints);

     void calcCoeffs(std::vector<double> initpos, std::vector<double> finalpos,
                     double maxVel, double maxAcc,
                     std::vector<double> init_vel = {},
                     std::vector<double> final_vel = {},
                     std::vector<double> init_accel = {},
                     std::vector<double> final_accel = {});

     void generatePathAndVel(double t, std::vector<double> &pos,
                             std::vector<double> &vel, std::vector<double> &acc);

     void printVec(std::vector<double> input);
     void printMat(std::vector<std::vector<double>> input);
     double getTime() { return _finalTime; }

     std::vector<std::vector<double>> &getPath() { return std::ref(_finalPath); }
     std::vector<std::vector<double>> &getVel() { return std::ref(_finalVel); }
     std::vector<std::vector<double>> &getAccel() { return std::ref(_finalAccel); }

     ~quint();

private:
     int _dof;
     int _waypts;
     double _finalTime;
     bool callcount{false};

     std::vector<double> _initpos, _finalpos;

     std::vector<std::vector<double>> _finalPath;
     std::vector<std::vector<double>> _finalVel;
     std::vector<std::vector<double>> _finalAccel;
     std::vector<std::vector<double>> _finalConstMat;

     // std::vector<double> _initial_pos;
     // std::vector<double> _final_pos;
     // std::vector<double> _inital_vel;
     // std::vector<double> _final_vel;

     Eigen::VectorXd _timeStep;
};

// #endif /* ADA958D4_E490_42C2_B895_E8BF9AB6FF71 */
