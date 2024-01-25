#pragma once
#include<iostream>
#include<vector>
#include<math.h>
#include <bits/stdc++.h>
class Scurve {
     private:
     double Ta_; // time for acceleration phase
     double Tv_; // time for constant velociy phase
     double Td_; // time for deceleation phase
     double finalTime_; // total time of trajectory
     double Tj1_; // time interval in which jerk is constant during acceleration
     double Tj2_; // time interval in which jerk is costant during deceleration
     double V0_; // initial velocity
     double V1_; // final velocity
     double Vmax_; // max Velocity
     double Vmin_;
     double Amax_; // max acceleration
     double Amin_;
     double Jmax_; // max Jerk
     double Jmin_;
     double Vlim_; // velocity limit
     double Alim_a_; // acceleration limit for acceleration phase
     double Alim_d_; // acceleration limit for deceleration phase
     double Y_; // gamma (ranges from 0-1)
     int dof_;  // degree's of freedom

     std::vector<std::vector<double>>finalCoeffMat_;

     std::vector<double>initPos_ , finalPos_;
     public:
     Scurve ( );
     Scurve ( int dof );
     bool calcCoeffs ( std::vector<double>initPos , std::vector<double>targetPos , double maxVel = 20 , double maxAcc = 20 , double maxJerk = 20 , std::vector<double>initVel = {} , std::vector<double>finalVel = {},bool degrees=true );
     bool generatePathAndVel ( double t , std::vector<double> &pos , std::vector<double> &vel , std::vector<double> &acc , std::vector<double> &jerk );

     ~Scurve ( );
     };


