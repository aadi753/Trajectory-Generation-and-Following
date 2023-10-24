#include <quinticMulitViapt.h>

QuinticMultiViapt::QuinticMultiViapt(int dof, int viaptTime, int finalTime, int waypoints, std::vector<std::vector<double>> waypointList)
{
     std::cout << "QUINTIC MULTI VIA POINT TRAJECTORY !!\n\n";
     _waypointList = waypointList;
     _waypts = waypoints;
     _dof = dof;
     _viaPtTime = viaptTime;
     _finalTime = finalTime;
     Eigen::VectorXd tStep = Eigen::VectorXd::Zero(_waypts);

     tStep = tStep.LinSpaced(_waypts, 0, finalTime);
     _timeStep = tStep;
     // std::cout << "TIME STEP: " << tStep << " " << tStep.size() << "\n\n";
}

void QuinticMultiViapt::calcCoeffs(std::vector<double> init_pos, std::vector<double> viaPoint, std::vector<double> final_pos, std::vector<double> init_vel, std::vector<double> final_vel, bool lastSegment, std::vector<double> init_accel, std::vector<double> final_accel)
{
     // std::cout << "out of constructor and in calcCoeff!!!!!!!!!!!!! \n";
     init_vel.resize(_dof);
     final_vel.resize(_dof);
     init_accel.resize(_dof);
     final_accel.resize(_dof);

     // Ax=B (statespace format).
     Eigen::MatrixXd A = Eigen::MatrixXd::Zero(12, 12);     // matrix having coeff of the constants in cubic eqn.
     Eigen::MatrixXd A_INV = Eigen::MatrixXd::Zero(12, 12); // inveresee of the above matrix.
     Eigen::VectorXd x = Eigen::VectorXd::Zero(12);         // the inital and final condition vector having inital and final position and velocity.
     Eigen::VectorXd B = Eigen::VectorXd::Zero(12);         // the vector having the constant to be found out.

     //* filling the matrix having coeff of cubic eqn.
     A(0, 0) = 1.0;

     A(1, 1) = 1.0;

     A(2, 2) = 2.0;

     A(3, 3) = 6.0;

     A(4, 0) = 1.0;
     A(4, 1) = _viaPtTime;
     A(4, 2) = pow((_viaPtTime), 2);
     A(4, 3) = pow((_viaPtTime), 3);
     A(4, 4) = pow((_viaPtTime), 4);
     A(4, 5) = pow((_viaPtTime), 5);

     A(5, 6) = 1.0;

     A(6, 6) = 1.0;
     A(6, 7) = (_finalTime - _viaPtTime);
     A(6, 8) = pow((_finalTime - _viaPtTime), 2);
     A(6, 9) = pow((_finalTime - _viaPtTime), 3);
     A(6, 10) = pow((_finalTime - _viaPtTime), 4);
     A(6, 11) = pow((_finalTime - _viaPtTime), 5);

     A(7, 7) = 1.0;
     A(7, 8) = 2 * (_finalTime - _viaPtTime);
     A(7, 9) = 3 * pow((_finalTime - _viaPtTime), 2);
     A(7, 10) = 4 * pow((_finalTime - _viaPtTime), 3);
     A(7, 11) = 5 * pow((_finalTime - _viaPtTime), 4);

     A(8, 8) = 2.0;
     A(8, 9) = 6 * (_finalTime - _viaPtTime);
     A(8, 10) = 12 * pow((_finalTime - _viaPtTime), 2);
     A(8, 11) = 20 * pow((_finalTime - _viaPtTime), 3);

     A(9, 1) = 1.0;
     A(9, 2) = 2 * _viaPtTime;
     A(9, 3) = 3 * pow((_viaPtTime), 2);
     A(9, 4) = 4 * pow((_viaPtTime), 3);
     A(9, 5) = 5 * pow((_viaPtTime), 4);
     A(9, 7) = -1.0;

     A(10, 2) = 2.0;
     A(10, 3) = 6 * _viaPtTime;
     A(10, 4) = 12 * pow((_viaPtTime), 2);
     A(10, 5) = 20 * pow((_viaPtTime), 3);
     A(10, 8) = -2.0;

     A(11, 3) = 6.0;
     A(11, 4) = 24 * _viaPtTime;
     A(11, 5) = 60 * pow((_viaPtTime), 2);
     A(11, 9) = -6.0;

     // std::cout << A << "\n\n";
     //*calculating inverse of the above matrix */
     A_INV = A.inverse();

     //* filling the "B" vector with the values of the different joints one by one and finding the coeff for all jonits and pushing them to the "_finalCoeffMat".

     std::vector<double> result;
     for (size_t i = 0; i < _dof; i++)
     {
          result.resize(x.size(), 0.0);

          B[0] = init_pos[i];
          B[1] = init_vel[i];
          B[2] = init_accel[i];
          B[3] = 0.0; // inital Jerk
          B[4] = viaPoint[i];
          B[5] = viaPoint[i];
          B[6] = final_pos[i];
          B[7] = final_vel[i];
          B[8] = final_accel[i];
          B[9] = 0.0;
          B[10] = 0.0;
          B[11] = 0.0;   // initial jerk for 2nd segment
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

     generatePathAndVel(_finalConstMat, _timeStep, lastSegment);
     _finalConstMat.clear();
}

void QuinticMultiViapt::generatePathAndVel(std::vector<std::vector<double>> totalCoeffMat, Eigen::VectorXd linSpacedTime, bool lastSegment)
{
     // std::cout << "out of calcCOeff and in generate path vel !!!!!!!!!! \n";

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
          if (t > _viaPtTime && !lastSegment)
               continue;
          for (auto ele : totalCoeffMat)
          {
               if (t <= _viaPtTime) // first segment
               {
                    posResult = (ele[0] * 1) + (ele[1] * t) + (ele[2] * t * t) + (ele[3] * t * t * t) + (ele[4] * t * t * t * t) + (ele[5] * t * t * t * t * t);

                    velResult = (ele[0] * 0) + (ele[1] * 1) + (ele[2] * 2 * t) + (ele[3] * 3 * t * t) + (ele[4] * 4 * t * t * t) + (ele[5] * 5 * t * t * t * t);

                    accelResult = (ele[0] * 0) + (ele[1] * 0) + (ele[2] * 2) + (ele[3] * 6 * t) + (ele[4] * 12 * t * t) + (ele[5] * 20 * t * t * t);
               }
               else
               {
                    if (lastSegment) // second segment.
                    {
                         posResult = (ele[6] * 1) + (ele[7] * (t - _viaPtTime)) + (ele[8] * pow((t - _viaPtTime), 2)) + (ele[9] * pow((t - _viaPtTime), 3)) + (ele[10] * pow((t - _viaPtTime), 4)) + (ele[11] * pow((t - _viaPtTime), 5));

                         velResult = (ele[6] * 0) + (ele[7] * 1) + (ele[8] * 2 * (t - _viaPtTime)) + (ele[9] * 3 * pow((t - _viaPtTime), 2)) + (ele[10] * 4 * pow((t - _viaPtTime), 3)) + (ele[11] * 5 * pow((t - _viaPtTime), 4));

                         accelResult = (ele[6] * 0) + (ele[7] * 0) + (ele[8] * 2) + (ele[9] * 6 * pow((t - _viaPtTime), 1)) + (ele[10] * 12 * pow((t - _viaPtTime), 2)) + (ele[11] * 20 * pow((t - _viaPtTime), 3));
                    }
               }
               jointPosVec.emplace_back(posResult);
               jointVelVec.emplace_back(velResult);
               jointAccelVec.emplace_back(accelResult);
          }

          //* print the vectors here to observe the values.
          //  printVec(jointPosVec);
          std::cout << jointPosVec[1] << '\n';

          _finalPath.emplace_back(jointPosVec);
          _finalVel.emplace_back(jointVelVec);
          _finalAccel.emplace_back(jointAccelVec);

          jointPosVec.clear();
          jointVelVec.clear();
          jointAccelVec.clear();
     }

     // *print the matrices here to observe the values
}

void QuinticMultiViapt::blendWaypoints(std::vector<std::vector<double>> wayptVector)
{
    
     int size = wayptVector.size();
     if (size == 3)
     {
          calcCoeffs(wayptVector[0], wayptVector[1], wayptVector[2],{},{},true);
     }
     else
     {
          for (size_t i = 0; i < wayptVector.size() - 3; i++)
          {
               if (i != 0)
               {
                    calcCoeffs(wayptVector[i], wayptVector[i + 1], wayptVector[i + 2], _finalVel[_finalVel.size() - 1], {}, false, _finalAccel[_finalAccel.size() - 1], {});
               }
               else
                    calcCoeffs(wayptVector[i], wayptVector[i + 1], wayptVector[i + 2]);

               // std::cout << i << "\n";
          }

          calcCoeffs(wayptVector[size - 3], wayptVector[size - 2], wayptVector[size - 1], _finalVel[_finalVel.size() - 1], {}, true, _finalAccel[_finalAccel.size() - 1], {});
     }
}

void QuinticMultiViapt::findCoeff(std::vector<double> init_pos, std::vector<double> final_pos, std::vector<double> waypoint, std::vector<double> init_vel, std::vector<double> final_vel, std::vector<double> init_accel, std::vector<double> final_accel)
{
     // std::cout << "inside findCOeff !!!!!!!!!!! \n";
     QuinticMultiViapt::blendWaypoints(_waypointList);
}

/*
 *@brief Destroy the Cubic Via Point::Cubic Via Point object
 *
 */

QuinticMultiViapt::~QuinticMultiViapt()
{
     // std::cout << "SAB KHATAM KARDIA BHAI :/ "<< "\n";
}

// ! HELPER FUNCTION TO PRINT THE VECTORS AND MATRICES. WILL BE REMOVE FROM HERE LATER. :)
void QuinticMultiViapt::printVec(std::vector<double> input)
{
     for (size_t i = 0; i < input.size(); i++)
     {
          std::cout << input[i] << " ";
     }
     std::cout << "\n\n";
}

void QuinticMultiViapt::printMat(std::vector<std::vector<double>> input)
{
     for (auto &ele : input)
     {
          for (auto &el : ele)
          {
               std::cout << el << " ";
          }
          std::cout << "\n";
     }
     std::cout << "\n";
}
