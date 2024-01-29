/**
 *@file Scurve.h
 * @author Aditya singh (aditya.singh@addverb.com)
 * @brief An optimal fast trajectory generator for N number of axis
 * @version 0.1
 * @date 2024-01-26
 *
 * @copyright Copyright (c) 2024
 *
 */


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
     double Vmin_; // min velocity
     double Amax_; // max acceleration
     double Amin_; // min acceleration
     double Jmax_; // max Jerk
     double Jmin_; // min jerk
     double Vlim_; // velocity limit
     double Alim_a_; // acceleration limit for acceleration phase
     double Alim_d_; // acceleration limit for deceleration phase
     double Y_; // gamma (ranges from 0-1)
     int dof_;  // degree's of freedom

     std::vector<std::vector<double>>finalCoeffMat_;
     std::vector<int>directionVec;
     std::vector<double>initPos_ , finalPos_;
     public:
     /**
      *@brief Construct a new Scurve object
      *
      */
     Scurve ( );
     Scurve ( int dof );
     /**
      *@brief computes the optimal time for all the axis based on given constrains.
      *
      * @param maxVel
      * @param maxAcc
      * @param maxJerk
      * @param degrees
      * @return true
      * @return false
      */
     bool calcCoeffs ( std::vector<double>initPos , std::vector<double>targetPos , double maxVel = 20 , double maxAcc = 20 , double maxJerk = 20 , bool degrees = true , std::vector<double>initVel = { 0 } , std::vector<double>finalVel = { 0 } );
/**
 *@brief generates the position,velocity,acceleration,jerk at the given time.
 *
 * @param t sampled time
 * @param pos position at given time
 * @param vel velocity at given time
 * @param acc acceleration at given time
 * @param jerk jerk at given time
 * @return true
 * @return false
 */
     bool generatePathAndVel ( double t , std::vector<double> &pos , std::vector<double> &vel , std::vector<double> &acc , std::vector<double> &jerk );

     ~Scurve ( );
     };


