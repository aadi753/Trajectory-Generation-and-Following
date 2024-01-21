#pragma once
#include<iostream>
#include<vector>
#include<trajectories.h>


class Scurve :public Trajectories {
     private:
     double Ta_; // time for acceleration phase
     double Tv_; // time for constant velociy phase
     double Td_; // time for deceleation phase
     double finalTime_; // total time of trajectory
     double Tj1_; // internal variable
     double Tj2_; // internal variable
     double V0_; // initial velocity
     double V1_; // final velocity
     double Vmax_; // max Velocity
     double Amax_; // max acceleration
     double Jmax_; // max Jerk
     double Vlim_; // velocity limit
     double Alim_a_; // acceleration limit for acceleration phase
     double Alim_d_; // acceleration limit for deceleration phase
     double Y_; // gamma (ranges from 0-1)
     int dof_;  // degree's of freedom

     public:
     Scurve ( );
     Scurve ( int dof );
     ~Scurve ( );
     };

Scurve::Scurve (/* args */ ) { }

Scurve::~Scurve ( ) { }
