#ifndef A8C5104A_9F79_474F_9A6E_8ED6725D6767
#define A8C5104A_9F79_474F_9A6E_8ED6725D6767

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <vector>
#include <trajectories/trajectories.h>

class Septic:public Trajectories
{

public:
     Septic(int dof, int finalTime, int waypoints);

     void calcCoeffs(std::vector<double> init_pos, std::vector<double> final_pos, std::vector<double> init_vel={}, std::vector<double> final_vel={}, std::vector<double> init_accel={}, std::vector<double> final_accel={},std::vector<double>init_jerk={},std::vector<double> final_jerk={});

     void generatePathAndVel(std::vector<std::vector<double>> totalCoeffMat, Eigen::VectorXd linSpacedTime);

     void printVec(std::vector<double> input);
     void printMat(std::vector<std::vector<double>> input);

     std::vector<std::vector<double>> &getPath() { return std::ref(_finalPath); }
     std::vector<std::vector<double>> &getVel() { return std::ref(_finalVel); }
     std::vector<std::vector<double>> &getAccel() { return std::ref(_finalAccel); }

     void findCoeff(std::vector<double> init_pos, std::vector<double> final_pos, std::vector<double> waypoint = {}, std::vector<double> init_vel = {}, std::vector<double> final_vel = {}, std::vector<double> init_accel = {}, std::vector<double> final_accel = {});

     ~Septic();

private:
     int _dof;
     int _waypts;
     double _finalTime;

     std::vector<std::vector<double>> _finalPath;
     std::vector<std::vector<double>> _finalVel;
     std::vector<std::vector<double>> _finalAccel;
     std::vector<std::vector<double>> _finalJerk;
     std::vector<std::vector<double>> _finalConstMat;

     // std::vector<double> _initial_pos;
     // std::vector<double> _final_pos;
     // std::vector<double> _inital_vel;
     // std::vector<double> _final_vel;

     Eigen::VectorXd _timeStep;
};

#endif /* A8C5104A_9F79_474F_9A6E_8ED6725D6767 */
