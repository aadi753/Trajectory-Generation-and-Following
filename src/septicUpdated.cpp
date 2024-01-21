#include <septicUpdated.h>

SEPTIC::SEPTIC ( ) {
     _dof = 6;
     }

SEPTIC::SEPTIC ( int dof ) {
  // std::cout << "SEPTIC TRAJECTORY !! \n\n";

     _dof = dof;
     }

void SEPTIC::calcCoeffs (
     std::vector<double> init_pos , std::vector<double> final_pos , double maxvel ,
     double maxacc , double maxjerk , bool degrees , std::vector<double> init_vel ,
     std::vector<double> final_vel , std::vector<double> init_accel ,
     std::vector<double> final_accel , std::vector<double> init_jerk ,
     std::vector<double> final_jerk ) {
     init_vel.resize ( _dof , 0.0 );
     final_vel.resize ( _dof , 0.0 );
     init_accel.resize ( _dof , 0.0 );
     final_accel.resize ( _dof , 0.0 );
     init_jerk.resize ( _dof , 0.0 );
     final_jerk.resize ( _dof , 0.0 );

     if ( degrees ) {
          maxvel = maxvel * ( M_PI / 180 );
          maxacc = maxacc * ( M_PI / 180 );
          maxjerk = maxjerk * ( M_PI / 180 );

          for ( size_t i = 0; i < _dof; i++ ) {
            // init_pos[i] = init_pos[i] * (M_PI / 180);
               final_pos [ i ] = final_pos [ i ] * ( M_PI / 180 );
               init_vel [ i ] = init_vel [ i ] * ( M_PI / 180 );
               init_accel [ i ] = init_accel [ i ] * ( M_PI / 180 );
               init_jerk [ i ] = init_jerk [ i ] * ( M_PI / 180 );
               final_vel [ i ] = final_vel [ i ] * ( M_PI / 180 );
               final_accel [ i ] = final_accel [ i ] * ( M_PI / 180 );
               final_jerk [ i ] = final_jerk [ i ] * ( M_PI / 180 );
               }
          }

          // TODO : find optimal time for trajectory

     std::vector<double> diffVec;
     for ( size_t i = 0; i < final_pos.size ( ); i++ ) {
          diffVec.emplace_back ( std::abs ( final_pos [ i ] - init_pos [ i ] ) );
          }

     double distance = ( *std::max_element ( diffVec.begin ( ) , diffVec.end ( ) ) );

     // std::cout << distance << "\n\n";
     // ! safety clamping of constrains if distance to travel is less

     _maxvel = maxvel;   // will be used to clamp the velocity for safety.

     //! NOTE: these numbers are calculated mathematically so DO NOT CHANGE THEM.
     double t1 = ( 35 * ( distance ) ) / ( 16 * maxvel );
     double t2 = sqrt ( ( ( 7.5132 * distance ) / ( maxacc ) ) );
     double t3 = cbrt ( ( 52.5 * distance ) / maxjerk );

     // std::cout << t1 << " " << t2 << " " << t3 << "\n";

     _finalTime = ceil ( std::max ( t1 , std::max ( t2 , t3 ) ) ) +
          0.5;   // additional buffer of 0.5 sec.

     if ( distance < 20 * ( M_PI / 180 ) && degrees ) {
          std::cout << "\n [SEPTIC]: CLAMPING!\n";
          _finalTime += 5;
          }
     std::cout << "FINAL TIME: " << _finalTime << "\n\n";

     //* Ax=B
     Eigen::MatrixXd A = Eigen::MatrixXd::Zero (
          8 , 8 );   // matrix having coeff of the constants in SEPTIC eqn.
     Eigen::MatrixXd A_INV =
          Eigen::MatrixXd::Zero ( 8 , 8 );   // inveresee of the above matrix.
     Eigen::VectorXd x = Eigen::VectorXd::Zero (
          8 );   // the inital and final condition vector having
                // inital and final position and velocity.
     Eigen::VectorXd B = Eigen::VectorXd::Zero (
          8 );   // the vector having the constant to be found out.

      //* filling the matrix having coeff of SEPTIC eqn.
     A ( 0 , 0 ) = 1.0;
     A ( 1 , 1 ) = 1.0;
     A ( 2 , 2 ) = 2.0;
     A ( 3 , 3 ) = 6.0;
     A ( 4 , 0 ) = 1.0;
     A ( 4 , 1 ) = _finalTime;
     A ( 4 , 2 ) = pow ( _finalTime , 2 );
     A ( 4 , 3 ) = pow ( _finalTime , 3 );
     A ( 4 , 4 ) = pow ( _finalTime , 4 );
     A ( 4 , 5 ) = pow ( _finalTime , 5 );
     A ( 4 , 6 ) = pow ( _finalTime , 6 );
     A ( 4 , 7 ) = pow ( _finalTime , 7 );
     A ( 5 , 1 ) = 1.0;
     A ( 5 , 2 ) = 2 * ( _finalTime );
     A ( 5 , 3 ) = 3 * pow ( _finalTime , 2 );
     A ( 5 , 4 ) = 4 * pow ( _finalTime , 3 );
     A ( 5 , 5 ) = 5 * pow ( _finalTime , 4 );
     A ( 5 , 6 ) = 6 * pow ( _finalTime , 5 );
     A ( 5 , 7 ) = 7 * pow ( _finalTime , 6 );
     A ( 6 , 2 ) = 2.0;
     A ( 6 , 3 ) = 6 * ( _finalTime );
     A ( 6 , 4 ) = 12 * pow ( _finalTime , 2 );
     A ( 6 , 5 ) = 20 * pow ( _finalTime , 3 );
     A ( 6 , 6 ) = 30 * pow ( _finalTime , 4 );
     A ( 6 , 7 ) = 42 * pow ( _finalTime , 5 );
     A ( 7 , 3 ) = 6.0;
     A ( 7 , 4 ) = 24 * ( _finalTime );
     A ( 7 , 5 ) = 60 * pow ( _finalTime , 2 );
     A ( 7 , 6 ) = 120 * pow ( _finalTime , 3 );
     A ( 7 , 7 ) = 210 * pow ( _finalTime , 4 );

     //*calculating inverse of the above matrix */
     A_INV = A.inverse ( );

     //* filling the "B" vector with the values of the different joints one by one
     // and finding the coeff for all jonits and pushing them to the
     //"_finalCoeffMat".

     if ( _finalConstMat.size ( ) != 0 && callCount )
          _finalConstMat.clear ( );

     std::vector<double> result;
     for ( size_t i = 0; i < _dof; i++ ) {
          result.resize ( x.size ( ) , 0.0 );

          B [ 0 ] = init_pos [ i ];
          B [ 1 ] = init_vel [ i ];
          B [ 2 ] = init_accel [ i ];
          B [ 3 ] = init_jerk [ i ];

          B [ 4 ] = final_pos [ i ];
          B [ 5 ] = final_vel [ i ];
          B [ 6 ] = final_accel [ i ];
          B [ 7 ] = final_jerk [ i ];

          x = A_INV * B;
          // std::cout << "X: " << x << " " << i << " " << "\n";

          // * filling the values in "x" into another vector that will be pushed to
          // the "_finalCoeffMat."

          for ( size_t i = 0; i < x.size ( ); i++ ) {
               result [ i ] = x [ i ];
               }

               // std::cout << "values of constants: "
               //           << "\n";

               // printVec(result);
          _finalConstMat.emplace_back ( result );
          result.clear ( );

          }   //! After this loop ends we'll have constants for all the joints in a
              //! matrix called "_finalCoeffMat".
     callCount = true;
     }

bool SEPTIC::generatePathAndVel ( double t , std::vector<double> &position ,
     std::vector<double> &velocity ,
     std::vector<double> &acceleration ,
     std::vector<double> &jerk ) {
     if ( t > _finalTime ) {
       // std::cout << " \n\n [SEPTIC]: ROBOT NOT ABLE TO KEEP UP WITH TRAJECTORY, "
       //              "EXITING!!\n\n ";
          for ( int i = 0; i < _dof; i++ ) {
               velocity [ i ] = 0;
               }
          return false;
          }

     std::vector<double> jointPosVec;
     std::vector<double> jointVelVec;
     std::vector<double> jointAccelVec;
     std::vector<double> jointJerkVec;

     {
       // std::cout << "index: " << i << std::endl;
     double posResult;
     double velResult;
     double accelResult;
     double jerkResult;

     for ( auto ele : _finalConstMat ) {
          posResult = ( ele [ 0 ] * 1 ) + ( ele [ 1 ] * t ) + ( ele [ 2 ] * t * t ) +
               ( ele [ 3 ] * t * t * t ) + ( ele [ 4 ] * t * t * t * t ) +
               ( ele [ 5 ] * t * t * t * t * t ) +
               ( ele [ 6 ] * t * t * t * t * t * t ) +
               ( ele [ 7 ] * t * t * t * t * t * t * t );

          velResult = ( ele [ 0 ] * 0 ) + ( ele [ 1 ] * 1 ) + ( ele [ 2 ] * 2 * t ) +
               ( ele [ 3 ] * 3 * t * t ) + ( ele [ 4 ] * 4 * t * t * t ) +
               ( ele [ 5 ] * 5 * t * t * t * t ) +
               ( ele [ 6 ] * 6 * t * t * t * t * t ) +
               ( ele [ 7 ] * 7 * t * t * t * t * t * t );

          accelResult = ( ele [ 0 ] * 0 ) + ( ele [ 1 ] * 0 ) + ( ele [ 2 ] * 2 ) +
               ( ele [ 3 ] * 6 * t ) + ( ele [ 4 ] * 12 * t * t ) +
               ( ele [ 5 ] * 20 * t * t * t ) + ( ele [ 6 ] * 30 * t * t * t * t ) +
               ( ele [ 7 ] * 42 * t * t * t * t * t );

          jerkResult = ( ele [ 0 ] * 0 ) + ( ele [ 1 ] * 0 ) + ( ele [ 2 ] * 0 ) + ( ele [ 3 ] * 6 ) +
               ( ele [ 4 ] * 24 * t ) + ( ele [ 5 ] * 60 * t * t ) +
               ( ele [ 6 ] * 120 * t * t * t ) + ( ele [ 7 ] * 210 * t * t * t * t );

          if ( std::abs ( velResult ) > _maxvel + 0.01 || std::isnan ( velResult ) ) {
               std::cout << " [SEPTIC]: VELOCITY EXCEEDED \n";
               velResult = 0;   // robot should not move if the velocity is not
               return false;
               }
               // respecting the limits.

          jointPosVec.emplace_back ( posResult );
          jointVelVec.emplace_back ( velResult );
          jointAccelVec.emplace_back ( accelResult );
          jointJerkVec.emplace_back ( jerkResult );
          }

          // printVec(jointPosVec);
     position = jointPosVec;
     velocity = jointVelVec;
     acceleration = jointAccelVec;
     jerk = jointJerkVec;
     }
     // printMat(_finalPath);
     return true;
     }

SEPTIC::~SEPTIC ( ) { }

// ! HELPER FUNCTION TO PRINT THE VECTORS AND MATRICES. WILL BE REMOVED LATER :)
void SEPTIC::printVec ( std::vector<double> input ) {
     for ( size_t i = 0; i < input.size ( ); i++ ) {
          std::cout << input [ i ] << "\t";
          }
     std::cout << "\n";
     }

void SEPTIC::printMat ( std::vector<std::vector<double>> input ) {
     for ( auto &ele : input ) {
          for ( auto &el : ele ) {
               std::cout << el << " ";
               }
          std::cout << "\n";
          }
     }
