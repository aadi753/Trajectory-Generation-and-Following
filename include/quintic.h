#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <vector>
#include <trajectories.h>

class Quintic:public Trajectories
{

public:
     /**
      * @brief Construct a new Quintic:: Quintic object
      *
      * @param dof
      * @param finalTime
      * @param waypoints
      */
     Quintic(int dof, int finalTime, int waypoints);

     void calcCoeffs(std::vector<double> init_pos, std::vector<double> final_pos, std::vector<double> init_vel={}, std::vector<double> final_vel={}, std::vector<double> init_accel={}, std::vector<double> final_accel={});

     void generatePathAndVel(std::vector<std::vector<double>> totalCoeffMat, Eigen::VectorXd linSpacedTime);

     void printVec(std::vector<double> input);
     void printMat(std::vector<std::vector<double>> input);

     std::vector<std::vector<double>> &getPath() { return std::ref(_finalPath); }
     std::vector<std::vector<double>> &getVel() { return std::ref(_finalVel); }
     std::vector<std::vector<double>> &getAccel() { return std::ref(_finalAccel); }

     void findCoeff(std::vector<double> init_pos, std::vector<double> final_pos, std::vector<double> waypoint = {}, std::vector<double> init_vel = {}, std::vector<double> final_vel = {}, std::vector<double> init_accel = {}, std::vector<double> final_accel = {});

     ~Quintic();

private:
     int _dof;
     int _waypts;
     double _finalTime;

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


