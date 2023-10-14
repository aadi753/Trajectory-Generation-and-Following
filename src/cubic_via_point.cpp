/**
 * @file cubic_via_point.cpp
 * @author Aditya Singh (aditya.in753@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-05-30
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <cubic_via_point.h>


CubicViaPoint::CubicViaPoint(int dof, int viaptTime, int finalTime, int waypoints)
{
     std::cout << "CUBIC VIA POINT TRAJECTORY !!\n\n";

     _waypts = waypoints;
     _dof = dof;
     _viaPtTime = viaptTime;
     _finalTime = finalTime;
     Eigen::VectorXd tStep = Eigen::VectorXd::Zero(_waypts);

     tStep = tStep.LinSpaced(_waypts, 0, finalTime);
     _timeStep = tStep;
     // std::cout << "TIME STEP: " << tStep << " " << tStep.size() << "\n\n";
}


void CubicViaPoint::calcCoeffs(std::vector<double> init_pos, std::vector<double> viaPoint, std::vector<double> final_pos, std::vector<double> init_vel, std::vector<double> final_vel)
{
     init_vel.resize(_dof, 0.0);
     final_vel.resize(_dof, 0.0);

     // Ax=B (statespace format).
     Eigen::MatrixXd A = Eigen::MatrixXd::Zero(8, 8);     // matrix having coeff of the constants in cubic eqn.
     Eigen::MatrixXd A_INV = Eigen::MatrixXd::Zero(8, 8); // inveresee of the above matrix.
     Eigen::VectorXd x = Eigen::VectorXd::Zero(8);        // the inital and final condition vector having inital and final position and velocity.
     Eigen::VectorXd B = Eigen::VectorXd::Zero(8);        // the vector having the constant to be found out.

     //* filling the matrix having coeff of cubic eqn.
     A(0, 0) = 1.0;

     A(1, 1) = 1.0;

     A(2, 0) = 1.0;
     A(2, 1) = _viaPtTime;
     A(2, 2) = pow(_viaPtTime, 2);
     A(2, 3) = pow(_viaPtTime, 3);

     A(3, 4) = 1.0;

     A(4, 4) = 1.0;
     A(4, 5) = (_finalTime - _viaPtTime);
     A(4, 6) = pow((_finalTime - _viaPtTime), 2);
     A(4, 7) = pow((_finalTime - _viaPtTime), 3);

     A(5, 5) = 1.0;
     A(5, 6) = 2 * (_finalTime - _viaPtTime);
     A(5, 7) = 3 * pow((_finalTime - _viaPtTime), 2);

     A(6, 1) = 1.0;
     A(6, 2) = 2 * (_viaPtTime);
     A(6, 3) = 3 * pow(_viaPtTime, 2);
     A(6, 5) = -1.0;

     A(7, 2) = 2.0;
     A(7, 3) = 6 * (_viaPtTime);
     A(7, 6) = -2.0;

     //*calculating inverse of the above matrix */
     A_INV = A.inverse();

     //* filling the "B" vector with the values of the different joints one by one and finding the coeff for all jonits and pushing them to the "_finalCoeffMat".

     std::vector<double> result;
     for (size_t i = 0; i < _dof; i++)
     {
          result.resize(x.size(), 0.0);

          B[0] = init_pos[i];
          B[1] = init_vel[i];
          B[2] = viaPoint[i];
          B[3] = viaPoint[i];
          B[4] = final_pos[i];
          B[5] = final_vel[i];
          B[6] = 0.0;
          B[7] = 0.0;

          x = A_INV * B; // * calculating the coefficients of cubic polynomial for all joints

          // * filling the values in "x" into another vector that will be pushed to the "_finalCoeffMat."

          for (size_t i = 0; i < x.size(); i++)
          {
               result[i] = x[i];
          }

          // printVec(result);

          _finalConstMat.emplace_back(result);
          result.clear();

     } //! After this loop ends we'll have constants for all the joints in a matrix called "_finalCoeffMat".

     generatePathAndVel(_finalConstMat, _timeStep);
}


void CubicViaPoint::generatePathAndVel(std::vector<std::vector<double>> totalCoeffMat, Eigen::VectorXd linSpacedTime)
{
     double t;
     std::vector<double> jointPosVec;
     std::vector<double> jointVelVec;
     std::vector<double> jointAccelVec;

     for (size_t i = 0; i < linSpacedTime.size(); i++)
     {
          t = linSpacedTime[i]; // time vaires from 0 to _finalTime.
          double posResult{0};
          double velResult{0};
          double accelResult{0};

          for (auto ele : totalCoeffMat) 
          {
               if (t < _viaPtTime) // first segment
               {
                    posResult = (ele[0] * 1) + (ele[1] * t) + (ele[2] * t * t) + (ele[3] * t * t * t);

                    velResult = (ele[0] * 0) + (ele[1] * 1) + (ele[2] * 2 * t) + (ele[3] * 3 * t * t);
                    accelResult = (ele[0] * 0) + (ele[1] * 0) + (ele[2] * 2) + (ele[3] * 6 * t);
               }
               else // second segment.
               {
                    posResult = (ele[4] * 1) + (ele[5] * (t - _viaPtTime)) + (ele[6] * pow((t - _viaPtTime), 2)) + (ele[7] * pow((t - _viaPtTime), 3));

                    velResult = (ele[4] * 0) + (ele[5] * 1) + (ele[6] * 2 * (t - _viaPtTime)) + (ele[7] * 3 * pow((t - _viaPtTime), 2));
                    accelResult = (ele[4] * 0) + (ele[5] * 0) + (ele[6] * 2) + (ele[7] * 6 * (t - _viaPtTime));
               }

               jointPosVec.emplace_back(posResult);
               jointVelVec.emplace_back(velResult);
               jointAccelVec.emplace_back(accelResult);
          }

          //* print the vectors here to observe the values.
          //  printVec(jointVelVec);
          // std::cout << jointVelVec[2] << '\n';

          _finalPath.emplace_back(jointPosVec);
          _finalVel.emplace_back(jointVelVec);
          _finalAccel.emplace_back(jointAccelVec);

          jointPosVec.clear();
          jointVelVec.clear();
          jointAccelVec.clear();
     }
     // *print the matrices here to observe the values
     // printMat(_finalPath);
}

void CubicViaPoint::findCoeff(std::vector<double> init_pos, std::vector<double> final_pos, std::vector<double> waypoint, std::vector<double> init_vel, std::vector<double> final_vel, std::vector<double> init_accel, std::vector<double> final_accel)
{
     CubicViaPoint::calcCoeffs(init_pos, waypoint, final_pos, init_vel, final_vel);
}

/*
 *@brief Destroy the Cubic Via Point::Cubic Via Point object
 *
 */

CubicViaPoint::~CubicViaPoint()
{
     // std::cout << "SAB KHATAM KARDIA BHAI :/ "
     //           << "\n";
}

// ! HELPER FUNCTION TO PRINT THE VECTORS AND MATRICES. WILL BE REMOVE FROM HERE LATER. :)
void CubicViaPoint::printVec(std::vector<double> input)
{
     for (size_t i = 0; i < input.size(); i++)
     {
          std::cout << input[i] << " ";
     }
     std::cout << "\n\n";
}

void CubicViaPoint::printMat(std::vector<std::vector<double>> input)
{
     for (auto &ele : input)
     {
          for (auto &el : ele)
          {
               std::cout << el << " ";
          }
          std::cout << "\n";
     }
}
