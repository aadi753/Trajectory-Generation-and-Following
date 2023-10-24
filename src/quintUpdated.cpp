#include<quintUpdated.h>

/**
 * @file quint.cpp
 * @author Aditya Singh (aditya.in753@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-05-31
 *
 * @copyright Copyright (c) 2023
 *
 */

quint::quint(int dof, int waypoints)
{
     // std::cout << "quint TRAJECTORY !! \n\n";
     // _waypts = waypoints;
     _dof = dof;
}

void quint::calcCoeffs(std::vector<double> initpos, std::vector<double> finalpos,
                       double maxVel, double maxAcc, std::vector<double> init_vel,
                       std::vector<double> final_vel, std::vector<double> init_accel,
                       std::vector<double> final_accel)
{
     init_vel.resize(_dof, 0.0);
     final_vel.resize(_dof, 0.0);

     init_accel.resize(_dof, 0.0);
     final_accel.resize(_dof, 0.0);

     std::vector<double> diffVec;
     for (size_t i = 0; i < finalpos.size(); i++)
     {
          diffVec.emplace_back(abs(finalpos[i] - initpos[i]));
     }

     double distance = *std::max_element(diffVec.begin(), diffVec.end());

     std::cout << distance << "\n\n";

     double t1 = 15 * (distance) / (8 * maxVel);
     double t2 = sqrt(((10 * sqrt(3)) * distance) / (3 * maxAcc));

     std::cout << t1 << " " << t2 << "\n";

     _finalTime = std::max(t1, t2);
     std::cout << _finalTime << "\n\n";

     // Eigen::VectorXd tStep = Eigen::VectorXd::Zero(_waypts);
     // tStep = tStep.LinSpaced(_waypts, 0, _finalTime);
     // _timeStep = tStep;

     //* Ax=B
     Eigen::MatrixXd A = Eigen::MatrixXd::Zero(
         6, 6); // matrix having coeff of the constants in quint eqn.
     Eigen::MatrixXd A_INV =
         Eigen::MatrixXd::Zero(6, 6); // inveresee of the above matrix.
     Eigen::VectorXd x =
         Eigen::VectorXd::Zero(6); // the inital and final condition vector having
                                   // inital and final position and velocity.
     Eigen::VectorXd B = Eigen::VectorXd::Zero(
         6); // the vector having the constant to be found out.

     //* filling the matrix having coeff of quint eqn.
     A(0, 0) = 1.0;
     A(1, 1) = 1.0;
     A(2, 2) = 2.0;
     A(3, 0) = 1.0;
     A(3, 1) = _finalTime;
     A(3, 2) = pow(_finalTime, 2);
     A(3, 3) = pow(_finalTime, 3);
     A(3, 4) = pow(_finalTime, 4);
     A(3, 5) = pow(_finalTime, 5);
     A(4, 1) = 1.0;
     A(4, 2) = 2.0 * (_finalTime);
     A(4, 3) = 3.0 * pow(_finalTime, 2);
     A(4, 4) = 4.0 * pow(_finalTime, 3);
     A(4, 5) = 5.0 * pow(_finalTime, 4);
     A(5, 2) = 2.0;
     A(5, 3) = 6.0 * _finalTime;
     A(5, 4) = 12.0 * pow(_finalTime, 2);
     A(5, 5) = 20.0 * pow(_finalTime, 3);

     //*calculating inverse of the above matrix */
     A_INV = A.inverse();

     //* filling the "B" vector with the values of the different joints one by one
     // and finding the coeff for all jonits and pushing them to the
     //"_finalCoeffMat".

     if (_finalConstMat.size() != 0 && callcount)
          _finalConstMat.clear();

     std::vector<double> result;
     for (size_t i = 0; i < _dof; i++)
     {
          result.resize(x.size(), 0.0);

          B[0] = initpos[i];
          B[1] = init_vel[i];
          B[2] = init_accel[i];
          B[3] = finalpos[i];
          B[4] = final_vel[i];
          B[5] = final_accel[i];

          x = A_INV * B;
          // std::cout << "X: " << x << " " << i << " " << "\n";

          // * filling the values in "x" into another vector that will be pushed to
          // the "_finalCoeffMat."

          for (size_t i = 0; i < x.size(); i++)
          {
               result[i] = x[i];
          }

          // std::cout << "values of constants: "
          //           << "\n";

          // printVec(result);
          // if (_finalConstMat.size() != 0) _finalConstMat.clear();
          _finalConstMat.emplace_back(result);
          result.clear();

     } //! After this loop ends we'll have constants for all the joints in a
       //! matrix called "_finalCoeffMat".

     // generatePathAndVel(_finalConstMat, t);
     callcount = true;
}

void quint::generatePathAndVel(double t, std::vector<double> &pos,
                               std::vector<double> &vel, std::vector<double> &acc)
{
     // double t;
     std::vector<double> jointPosVec;
     std::vector<double> jointVelVec;
     std::vector<double> jointAccelVec;

     {
          double posResult;
          double velResult;
          double accelResult;

          for (auto ele : _finalConstMat)
          {
               posResult = (ele[0] * 1) + (ele[1] * t) + (ele[2] * t * t) +
                           (ele[3] * t * t * t) + (ele[4] * t * t * t * t) +
                           (ele[5] * t * t * t * t * t);

               velResult = (ele[0] * 0) + (ele[1] * 1) + (ele[2] * 2 * t) +
                           (ele[3] * 3 * t * t) + (ele[4] * 4 * t * t * t) +
                           (ele[5] * 5 * t * t * t * t);

               accelResult = (ele[0] * 0) + (ele[1] * 0) + (ele[2] * 2) +
                             (ele[3] * 6 * t) + (ele[4] * 12 * t * t) +
                             (ele[5] * 20 * t * t * t);

               jointPosVec.emplace_back(posResult);
               jointVelVec.emplace_back(velResult);
               jointAccelVec.emplace_back(accelResult);
          }

          // printVec(jointPosVec);
          // std::cout << jointPosVec[2] << "\n";

          // _finalPath.emplace_back(jointPosVec);
          // _finalVel.emplace_back(jointVelVec);
          // _finalAccel.emplace_back(jointAccelVec);
          for (size_t i = 0; i < _dof; i++)
          {
               pos[i] = jointPosVec[i];
               vel[i] = jointVelVec[i];
               acc[i] = jointAccelVec[i];
               /* code */
          }
     }
     // printMat(_finalPath);
}

quint::~quint()
{
     
}

// ! HELPER FUNCTION TO PRINT THE VECTORS AND MATRICES. WILL BE REMOVED LATER :)
void quint::printVec(std::vector<double> input)
{
     for (size_t i = 0; i < input.size(); i++)
     {
          std::cout << std::fixed << input[i] << " ";
     }
     std::cout << "\n\n";
}

void quint::printMat(std::vector<std::vector<double>> input)
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
