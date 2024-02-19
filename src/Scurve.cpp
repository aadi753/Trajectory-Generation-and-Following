#include <Scurve.h>

Scurve::Scurve ( ) { }

Scurve::Scurve ( int dof  ) : dof_ ( dof ) {
     std::cout << "Scurve Traj\n";
     }
int Scurve::calcCoeffs (
     std::vector<double> initPos , std::vector<double> targetPos , double maxVel ,
     double maxAcc , double maxJerk , bool degrees , std::vector<double> initVel ,
     std::vector<double> finalVel , std::vector<double> initAccel ,
     std::vector<double> finalAccel , std::vector<double> initJerk ,
     std::vector<double> finalJerk ) {
     if ( maxVel > hardVelLimit_ || maxAcc > hardAccLimit_ ||
          maxJerk > hardJerkLimit_ ) {
          std::cout << "\n THE INPUT PARAMETERS ARE EXCEEDING THE HARD LIMITS SET "
               "FOR THE ROBOT. EXITING .....\n";
          return 1;
          }
     if ( degrees ) {
          Vmax_ = maxVel * ( M_PI / 180 );
          Amax_ = maxAcc * ( M_PI / 180 );
          Jmax_ = maxJerk * ( M_PI / 180 );
          Vmin_ = -Vmax_;
          Amin_ = -Amax_;
          Jmin_ = -Jmax_;
          V0_ = initVel [ 0 ] * ( M_PI / 180 );
          V1_ = finalVel [ 0 ] * ( M_PI / 180 );

          for ( size_t i = 0; i < targetPos.size ( ); i++ ) {
               targetPos [ i ] = targetPos [ i ] * ( M_PI / 180 );
               }

          initPos_ = initPos;
          finalPos_ = targetPos;

          }
     else {
          Vmax_ = maxVel;
          Amax_ = maxAcc;
          Jmax_ = maxJerk;
          Vmin_ = -Vmax_;
          Amin_ = -Amax_;
          Jmin_ = -Jmax_;
          initPos_ = initPos;
          finalPos_ = targetPos;
          V0_ = initVel [ 0 ];
          V1_ = finalVel [ 0 ];
          }

     int maxDistIndex;
     std::vector<double> diffVec;
     if ( directionVec.size ( ) != 0 )
          directionVec.clear ( );

     for ( size_t i = 0; i < targetPos.size ( ); i++ ) {
          diffVec.emplace_back ( std::abs ( targetPos [ i ] - initPos [ i ] ) );
          if ( initPos [ i ] > targetPos [ i ] ) {
               directionVec.emplace_back ( -1 );

               }
          else
               directionVec.emplace_back ( 1 );
          }

     double displacement = *std::max_element ( diffVec.begin ( ) , diffVec.end ( ) );
     auto it = std::find ( diffVec.begin ( ) , diffVec.end ( ) , displacement );
     maxDistIndex = it - diffVec.begin ( );

     // std::cout << "max Displacement: " << displacement << "\n";
     // std::cout << "maxDistance index: " << maxDistIndex << "\n";

     double TJ_star = std::min ( sqrt ( abs ( V1_ - V0_ ) / Jmax_ ) , Amax_ / Jmax_ );

     // check for the feasibility of the trajectory.
     if ( TJ_star = Amax_ / Jmax_ ) {
          if ( displacement >
               0.5 * ( V0_ + V1_ ) * ( TJ_star + ( abs ( V0_ + V1_ ) / Amax_ ) ) ) {
               std::cout << "Traj is feasable and a phase of 0 jerk may exist... \n";
               }
          }
     else if ( TJ_star < Amax_ / Jmax_ ) {
          if ( displacement > TJ_star * ( V0_ + V1_ ) ) {
               std::cout << "Traj is feasable... \n";
               }
          }
     else {
          std::cout << "optimal Trajectory not possible consider changing the "
               "control parameters...!\n";
          return 1;
          }

          // assuming case 1:
     Vlim_ = Vmax_;

     if ( ( Vmax_ - V0_ ) * Jmax_ < pow ( Amax_ , 2 ) ) {
          std::cout << "Amax will not be reached..! \n";
          Tj1_ = sqrt ( ( Vmax_ - V0_ ) / Jmax_ );
          Ta_ = 2 * Tj1_;
          // std::cout << "Tj1: " << Tj1_ << " || " << "Ta: " << Ta_ << "\n";
          }
     else {
          Tj1_ = Amax_ / Jmax_;
          Ta_ = Tj1_ + ( ( Vmax_ - V0_ ) / Amax_ );
          // std::cout << "Tj1: " << Tj1_ << " || " << "Ta: " << Ta_ << "\n";
          }

     if ( ( Vmax_ - V1_ ) * Jmax_ < pow ( Amax_ , 2 ) ) {
          std::cout << "Amin will not be reached..!\n";
          Tj2_ = sqrt ( ( Vmax_ - V1_ ) / Jmax_ );
          Td_ = 2 * Tj2_;
          // std::cout << "Tj2: " << Tj2_ << " || " << "Td: " << Td_ << "\n";

          }
     else {
          Tj2_ = Amax_ / Jmax_;
          Td_ = Tj2_ + ( ( Vmax_ - V1_ ) / Amax_ );
          // std::cout << "Tj2: " << Tj2_ << " || " << "Td: " << Td_ << "\n";
          }
          // now we can compute the time for constant velocity phase

     Tv_ = ( displacement / Vmax_ ) - ( ( Ta_ / 2 ) * ( 1 + V0_ / Vmax_ ) ) -
          ( ( Td_ / 2 ) * ( 1 + V1_ / Vmax_ ) );
    // std::cout << "time for const velocity segment: " << Tv_<<"\n";
     if ( Tv_ >
          0 )   // means our assumption of vlim=Vmax was right and can continue with the above values
          {
          std::cout << "max vel will be reached.. \n";
          }

     else {   // means our assumption of Vlim=Vmax was wrong and we have to recalculate using case 2.
          Tv_ = 0;
          // std::cout << "\ncase2:\n";
          double delta =
               ( pow ( Amax_ , 4 ) / pow ( Jmax_ , 2 ) ) + ( 2 * ( pow ( V0_ , 2 ) + pow ( V1_ , 2 ) ) ) +
               Amax_ * ( 4 * (displacement)-( ( 2 * ( Amax_ / Jmax_ ) ) * ( V0_ + V1_ ) ) );

          Tj1_ = Tj2_ = Amax_ / Jmax_;
          Ta_ = ( ( pow ( Amax_ , 2 ) / Jmax_ ) - ( 2 * V0_ ) + sqrt ( delta ) ) / ( 2 * Amax_ );
          Td_ = ( ( pow ( Amax_ , 2 ) / Jmax_ ) - ( 2 * V1_ ) + sqrt ( delta ) ) / ( 2 * Amax_ );

          if ( Ta_ < 2 * Tj1_ || Td_ < 2 * Tj1_ ) {
               for ( Y_ = 1; Y_ > 0; Y_ -= 0.01 ) {
                 // std::cout << Y_ << "\n";
                    Amax_ = Y_ * Amax_;
                    delta =
                         ( pow ( Amax_ , 4 ) / pow ( Jmax_ , 2 ) ) +
                         ( 2 * ( pow ( V0_ , 2 ) + pow ( V1_ , 2 ) ) ) +
                         Amax_ * ( 4 * (displacement)-( 2 * ( Amax_ / Jmax_ ) * ( V0_ + V1_ ) ) );

                    Tj1_ = Tj2_ = Amax_ / Jmax_;
                    Ta_ = ( ( pow ( Amax_ , 2 ) / Jmax_ ) - ( 2 * V0_ ) + sqrt ( delta ) ) / ( 2 * Amax_ );
                    Td_ = ( ( pow ( Amax_ , 2 ) / Jmax_ ) - ( 2 * V1_ ) + sqrt ( delta ) ) / ( 2 * Amax_ );
                    if ( Ta_ > 2 * Tj1_ && Td_ > 2 * Tj1_ ) {
                      // std::cout << "auto adjustments done..\n";
                         break;
                         }
                    else if ( Ta_ < 0 ) {
                         Ta_ = 0;
                         Tj1_ = 0;
                         Td_ = ( 2 * displacement ) / ( V0_ + V1_ );
                         Tj2_ = ( ( Jmax_ * displacement ) -
                              ( sqrt ( Jmax_ * ( Jmax_ * ( pow ( displacement , 2 ) ) +
                                   ( pow ( ( V0_ + V1_ ) , 2 ) * ( V1_ - V0_ ) ) ) ) ) ) /
                              ( Jmax_ * ( V0_ + V1_ ) );
                         break;
                         }
                    else if ( Td_ < 0 ) {
                         Td_ = 0;
                         Tj2_ = 0;
                         Ta_ = ( 2 * displacement ) / ( V0_ + V1_ );
                         Tj1_ = ( ( Jmax_ * displacement ) -
                              ( sqrt ( Jmax_ * ( Jmax_ * ( pow ( displacement , 2 ) ) +
                                   ( pow ( ( V0_ + V1_ ) , 2 ) * ( V1_ - V0_ ) ) ) ) ) ) /
                              ( Jmax_ * ( V0_ + V1_ ) );
                         break;
                         }
                    }
               }
          }

     Alim_a_ = Jmax_ * Tj1_;
     Alim_d_ = -Jmax_ * Tj2_;
     Vlim_ = V1_ - ( Td_ - Tj2_ ) * Alim_d_;
     if ( Vlim_ > maxVel ) {
          std::cout << "VELOCITY OR ACCELERATIONS LIMITS EXCEEDED!..\n";
          return 1;
          }
     std::cout << "Ta: " << Ta_ << " || "
          << "Td: " << Td_ << " || "
          << "Tv: " << Tv_ << " || "
          << "Tj1_: " << Tj1_ << " || "
          << "Tj2: " << Tj2_ << " || "
          << "Alim_a: " << Alim_a_ << " || "
          << "Alim_d: " << Alim_d_ << " || "
          << "Vlim_: " << Vlim_ << "\n";

     finalTime_ = ( Ta_ + Tv_ + Td_ );
     std::cout << "finalTime: " << finalTime_ << "\n";

     std::vector<double> maxDisplacementCoeffs;
     maxDisplacementCoeffs.emplace_back ( initPos [ maxDistIndex ] *
          directionVec [ maxDistIndex ] );
     maxDisplacementCoeffs.emplace_back (
          targetPos [ maxDistIndex ] *
          directionVec [ maxDistIndex ] );   //ta ,tj ,vmax,amax,jmax
     maxDisplacementCoeffs.emplace_back ( Ta_ );
     maxDisplacementCoeffs.emplace_back ( Td_ );
     maxDisplacementCoeffs.emplace_back ( Tv_ );
     maxDisplacementCoeffs.emplace_back ( Tj1_ );
     maxDisplacementCoeffs.emplace_back ( Tj2_ );
     maxDisplacementCoeffs.emplace_back ( Vlim_ );
     maxDisplacementCoeffs.emplace_back ( Alim_a_ );
     maxDisplacementCoeffs.emplace_back ( Alim_d_ );
     maxDisplacementCoeffs.emplace_back ( Jmax_ );
     maxDisplacementCoeffs.emplace_back ( Jmin_ );

     std::vector<double> res;
     double
          alpha = 0.4 ,
          beta =
          0.2;   //? do not change these values unless you know what they do!!.
     if ( finalCoeffMat_.size ( ) != 0 )
          finalCoeffMat_.clear ( );
     for ( size_t i = 0; i < dof_; i++ ) {
          if ( i == maxDistIndex ) {
               res = maxDisplacementCoeffs;

               }
          else {
               Vmax_ = ( diffVec [ i ] ) / ( ( 1 - alpha ) * finalTime_ );
               Amax_ =
                    diffVec [ i ] / ( alpha * ( 1 - alpha ) * ( 1 - beta ) * pow ( finalTime_ , 2 ) );
               Jmax_ = diffVec [ i ] / ( pow ( alpha , 2 ) * beta * ( 1 - alpha ) * ( 1 - beta ) *
                    pow ( finalTime_ , 3 ) );
               Vlim_ = Vmax_;
               Alim_a_ = Amax_;
               Alim_d_ = -Alim_a_;
               Jmin_ = -Jmax_;
               Ta_ = alpha * finalTime_;
               Td_ = Ta_;
               Tj1_ = Tj2_ = beta * Ta_;
               Tv_ = finalTime_ - ( 2 * Ta_ );

               res.emplace_back ( initPos [ i ] * directionVec [ i ] );
               res.emplace_back ( targetPos [ i ] *
                    directionVec [ i ] );   //ta ,tj ,vmax,amax,jmax
               res.emplace_back ( Ta_ );
               res.emplace_back ( Td_ );
               res.emplace_back ( Tv_ );
               res.emplace_back ( Tj1_ );
               res.emplace_back ( Tj2_ );
               res.emplace_back ( Vlim_ );
               res.emplace_back ( Alim_a_ );
               res.emplace_back ( Alim_d_ );
               res.emplace_back ( Jmax_ );
               res.emplace_back ( Jmin_ );
               }

          finalCoeffMat_.emplace_back ( res );
          res.clear ( );
          }

     return 0;
     }
int Scurve::generatePathAndVel ( double t , std::vector<double> &Pos ,
     std::vector<double> &Vel ,
     std::vector<double> &Acc ,
     std::vector<double> &Jerk ) {
     if ( t > finalTime_ ) {
          std::cout << "Trajectory time violation, exiting... \n";
          return 1;
          }
     double position;
     double velocity;
     double acceleration;
     double jerk;
     int i = 0;
     std::vector<double> posi , velo , accel , jer;
     // acceleration phase
     for ( auto &ele : finalCoeffMat_ ) {
          double Ta_ = ele [ 2 ];
          double Td_ = ele [ 3 ];
          double Tv_ = ele [ 4 ];
          double Tj1_ = ele [ 5 ];
          double Tj2_ = ele [ 6 ];
          double Vlim_ = ele [ 7 ];
          double Alim_a_ = ele [ 8 ];
          double Alim_d_ = ele [ 9 ];
          double Jmax_ = ele [ 10 ];
          double Jmin_ = ele [ 11 ];

          if ( t >= 0 && t < Tj1_ ) {
               position = ele [ 0 ] + ( V0_ * t ) + ( Jmax_ * pow ( t , 3 ) ) / 6;
               velocity = V0_ + ( Jmax_ * pow ( t , 2 ) ) / 2;
               acceleration = Jmax_ * t;
               jerk = Jmax_;
               }
          else if ( t >= Tj1_ && t < ( Ta_ - Tj1_ ) ) {
               position =
                    ele [ 0 ] + ( V0_ * t ) +
                    ( Alim_a_ / 6 ) * ( ( 3 * pow ( t , 2 ) ) - ( 3 * Tj1_ * t ) + pow ( Tj1_ , 2 ) );
               velocity = V0_ + Alim_a_ * ( t - ( Tj1_ / 2 ) );
               acceleration = Jmax_ * Tj1_;
               jerk = 0;
               }
          else if ( t >= ( Ta_ - Tj1_ ) && t < Ta_ ) {
               position = ele [ 0 ] + ( ( Vlim_ + V0_ ) * ( Ta_ / 2 ) ) - ( Vlim_ * ( Ta_ - t ) ) -
                    ( Jmin_ * ( pow ( ( Ta_ - t ) , 3 ) ) ) / 6;
               velocity = Vlim_ + ( Jmin_ * ( pow ( ( Ta_ - t ) , 2 ) ) ) / 2;
               acceleration = -Jmin_ * ( Ta_ - t );
               jerk = Jmin_;
               }

               // constant velocity phase
          else if ( t >= Ta_ && t <= ( Ta_ + Tv_ ) ) {
               position = ele [ 0 ] + ( ( Vlim_ + V0_ ) * ( Ta_ / 2 ) ) + ( Vlim_ * ( t - Ta_ ) );
               velocity = Vlim_;
               acceleration = 0;
               jerk = 0;
               }

               // deceleration phase
          else if ( t >= ( finalTime_ - Td_ ) && t < ( finalTime_ - Td_ + Tj2_ ) ) {
               position = ele [ 1 ] - ( ( Vlim_ + V1_ ) * ( Td_ / 2 ) ) +
                    ( Vlim_ * ( t - finalTime_ + Td_ ) ) -
                    ( Jmax_ * ( pow ( ( t - finalTime_ + Td_ ) , 3 ) ) ) / 6;
               velocity = Vlim_ - ( Jmax_ * ( pow ( ( t - finalTime_ + Td_ ) , 2 ) ) ) / 2;
               acceleration = Jmin_ * ( t - finalTime_ + Td_ );
               jerk = Jmin_;
               }
          else if ( t >= ( finalTime_ - Td_ + Tj2_ ) && t < ( finalTime_ - Tj2_ ) ) {
               position = ele [ 1 ] - ( ( Vlim_ + V1_ ) * ( Td_ / 2 ) ) +
                    ( Vlim_ * ( t - finalTime_ + Td_ ) ) +
                    ( ( Alim_d_ / 6 ) *
                         ( 3 * ( pow ( ( t - finalTime_ + Td_ ) , 2 ) ) -
                              ( 3 * Tj2_ * ( t - finalTime_ + Td_ ) ) + pow ( Tj2_ , 2 ) ) );
               velocity = Vlim_ + Alim_d_ * ( t - finalTime_ + Td_ - ( Tj2_ / 2 ) );
               acceleration = Alim_d_;
               jerk = 0;
               }
          else if ( t >= ( finalTime_ - Tj2_ ) && t <= finalTime_ ) {
               position = ele [ 1 ] - V1_ * ( finalTime_ - t ) -
                    ( Jmax_ * ( pow ( ( finalTime_ - t ) , 3 ) ) ) / 6;
               velocity = V1_ + ( Jmax_ * ( pow ( ( finalTime_ - t ) , 2 ) ) ) / 2;
               acceleration = Jmin_ * ( finalTime_ - t );
               jerk = Jmax_;
               }

          if ( ( std::abs ( velocity ) > ( std::abs ( Vlim_ ) + 0.01 ) ) ||
               std::isnan ( velocity ) ) {
               std::cout << " [S-curve]: VELOCITY EXCEEDED \n";
               velocity = 0;   // robot should not move if the velocity is not
               return 1;
               }
          posi.emplace_back ( position * directionVec [ i ] );
          velo.emplace_back ( velocity * directionVec [ i ] );
          accel.emplace_back ( acceleration * directionVec [ i ] );
          jer.emplace_back ( jerk * directionVec [ i ] );
          i++;
          }
     Pos = posi;
     Vel = velo;
     Acc = accel;
     Jerk = jer;

     return 0;
     }

int Scurve::getPrecomputedTraj ( std::vector<double> initPos ,
     std::vector<double> targetPos , double maxVel ,
     double maxAcc , double maxJerk , bool degrees ,
     std::vector<double> initVel ,
     std::vector<double> finalVel ) {
     if ( initPos.size ( ) != dof_ || targetPos.size ( ) != dof_ )
          return 1;

     calcCoeffs ( initPos , targetPos , maxVel , maxAcc , maxJerk , degrees );
     if ( pathMat_.size ( ) != 0 )
          pathMat_.clear ( );
     Eigen::VectorXd t;
     t = t.LinSpaced ( 50 , 0 , finalTime_ );
     std::vector<double> pos , vel , acc , j;
     for ( int i = 0; i < t.size ( ); i++ ) {
          generatePathAndVel ( t [ i ] , pos , vel , acc , j );
          pathMat_.emplace_back ( pos );
          }
     return 0;
     }

Scurve::~Scurve ( ) { }
