#include<Scurve.h>

Scurve::Scurve ( ) {
     std::cout << "Scurve Traj\n";
     }
Scurve::Scurve ( int dof ) :dof_ ( dof ) { }

bool Scurve::calcCoeff ( std::vector<double>targetPos , double maxVel , double maxAcc , double maxJerk , std::vector<double>initVel , std::vector<double>finalVel ) {
     Vmax_ = maxVel;
     Amax_ = maxAcc;
     Jmax_ = maxJerk;
     Vmin_ = -Vmax_;
     Amin_ = -Amax_;
     Jmin_ = -Jmax_;

     V0_ = initVel [ 0 ];
     V1_ = finalVel [ 0 ];

     std::vector<double>initPos = { 0 };
     double displacement = targetPos [ 0 ] - initPos [ 0 ];

     double TJ_star = std::min ( sqrt ( abs ( V1_ - V0_ ) / Jmax_ ) , Amax_ / Jmax_ );

     // check for the feasablity of the trajectory. 
     if ( TJ_star = Amax_ / Jmax_ ) {
          if ( displacement > 0.5 * ( V0_ + V1_ ) * ( TJ_star + ( abs ( V0_ + V1_ ) / Amax_ ) ) ) {

               std::cout << "Traj is feasable and a phase of 0 jerk may exist... \n";
               }
          }
     else if ( TJ_star < Amax_ / Jmax_ ) {
          if ( displacement > TJ_star * ( V0_ + V1_ ) ) {

               std::cout << "Traj is feasable... \n";
               }
          }
     else {
          std::cout << "optimal Trajectory not possible consider changing the control parameters...!\n";
          return false;
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

     Tv_ = ( displacement / Vmax_ ) - ( ( Ta_ / 2 ) * ( 1 + V0_ / Vmax_ ) ) - ( ( Td_ / 2 ) * ( 1 + V1_ / Vmax_ ) );
     // std::cout << "time for const velocity segment: " << Tv_<<"\n";
     if ( Tv_ > 0 ) // means our assumption of vlim=Vmax was right and can continue with the above values 
          {
          std::cout << "max vel will be reached.. \n";
          }



     else { // means our assumption of Vlim=Vmax was wrong and we have to recalculate using case 2.
          Tv_ = 0;
          std::cout << "\ncase2:\n";
          double delta = ( pow ( Amax_ , 4 ) / pow ( Jmax_ , 2 ) ) + ( 2 * ( pow ( V0_ , 2 ) + pow ( V1_ , 2 ) ) ) + Amax_ * ( 4 * (displacement)-( ( 2 * ( Amax_ / Jmax_ ) ) * ( V0_ + V1_ ) ) );
          std::cout << "DELTA: " << delta << "\n";

          Tj1_ = Tj2_ = Amax_ / Jmax_;
          Ta_ = ( ( pow ( Amax_ , 2 ) / Jmax_ ) - ( 2 * V0_ ) + sqrt ( delta ) ) /( 2 * Amax_);
          Td_ = ( ( pow ( Amax_ , 2 ) / Jmax_ ) - ( 2 * V1_ ) + sqrt ( delta ) ) / (2 * Amax_);

          if ( Ta_ < 2 * Tj1_ || Td_ < 2 * Tj1_ ) {
               for ( Y_ = 1; Y_ > 0; Y_ -= 0.01 ) {
                    // std::cout << Y_ << "\n";
                    Amax_ = Y_ * Amax_;
                    delta = ( pow ( Amax_ , 4 ) / pow ( Jmax_ , 2 ) ) + ( 2 * ( pow ( V0_ , 2 ) + pow ( V1_ , 2 ) ) ) + Amax_ * ( 4 * (displacement)-( 2 * ( Amax_ / Jmax_ ) * ( V0_ + V1_ ) ) );

                    Tj1_ = Tj2_ = Amax_ / Jmax_;
                    Ta_ = ( ( pow ( Amax_ , 2 ) / Jmax_ ) - ( 2 * V0_ ) + sqrt ( delta ) ) / (2 * Amax_);
                    Td_ = ( ( pow ( Amax_ , 2 ) / Jmax_ ) - ( 2 * V1_ ) + sqrt ( delta ) ) / (2 * Amax_);
                    if ( Ta_ > 2 * Tj1_ && Td_ > 2 * Tj1_ ) {
                         std::cout << "auto adjustments done..\n";
                         break;
                         }
                    else if ( Ta_ < 0 ) {
                         Ta_ = 0;
                         Tj1_ = 0;
                         Td_ = ( 2 * displacement ) / ( V0_ + V1_ );
                         Tj2_ = ( ( Jmax_ * displacement ) - ( sqrt ( Jmax_ * ( Jmax_ * ( pow ( displacement , 2 ) ) + ( pow ( ( V0_ + V1_ ) , 2 ) * ( V1_ - V0_ ) ) ) ) ) ) / (Jmax_ * ( V0_ + V1_ ));
                         break;
                         }
                    else if ( Td_ < 0 ) {
                         Td_ = 0;
                         Tj2_ = 0;
                         Ta_ = ( 2 * displacement ) / ( V0_ + V1_ );
                         Tj1_ = ( ( Jmax_ * displacement ) - ( sqrt ( Jmax_ * ( Jmax_ * ( pow ( displacement , 2 ) ) + ( pow ( ( V0_ + V1_ ) , 2 ) * ( V1_ - V0_ ) ) ) ) ) ) / (Jmax_ * ( V0_ + V1_ ));
                         break;
                         }

                    }


               }
          // if ( Ta_ < 2 * Tj1_ || Td_ < 2 * Tj1_ )
          //      std::cout << "auto adjustment failed update the control parameters.\n";
          }
          Alim_a_ = Jmax_ * Tj1_;
          Alim_d_ = -Jmax_ * Tj2_;
          Vlim_ = V1_ - ( Td_ - Tj2_ ) * Alim_d_;

          std::cout << "Ta: " << Ta_ << " || " << "Td: " << Td_ << " || " << "Tv: " << Tv_ << " || " << "Tj1_: " << Tj1_ << " || " << "Tj2: " << Tj2_ << " || " << "Alim_a: " << Alim_a_ << " || " << "Alim_d: " << Alim_d_ << " || " << "Vlim_: " << Vlim_ << "\n";



     return true;
     }
bool Scurve::generatePathAndVel ( double t , std::vector<double> &pos , std::vector<double> &vel , std::vector<double> &acc , std::vector<double> &jerk ) {
     return true;
     }


Scurve::~Scurve ( ) {
     
     }