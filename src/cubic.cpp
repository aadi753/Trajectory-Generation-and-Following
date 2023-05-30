/**
 * @file cubic.cpp
 * @author Aditya Singh (aditya.in753@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-05-31
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <cubic.h>

/**
 * @brief Construct a new Cubic::Cubic object
 *
 * @param dof no. of joints for which trajectory is needed.
 * @param finalTime total time of trajectory.
 * @param waypoints no. of waypoints to be generated from inital to final point.
 */
Cubic::Cubic(int dof, int finalTime, int waypoints)
{
     _waypts = waypoints;
     _dof = dof;
     _finalTime = finalTime;
     Eigen::VectorXd tStep = Eigen::VectorXd::Zero(_waypts);

     tStep = tStep.LinSpaced(_waypts, 0, finalTime);
     _timeStep = tStep;
     std::cout << "TIME STEP: " << tStep << " " << tStep.size() << "\n\n";
}

/**
 * @brief calculates the coefficients of the cubic polynomial that are to be used for generating the path and the velocities.
 *
 * @param init_pos vector of intial joint positions.
 * @param final_pos vector of final joint positions.
 * @param init_vel vector of intial joint velocities.
 * @param final_vel vector of final joint velocities.
 */
void Cubic::calcCoeffs(std::vector<double> init_pos, std::vector<double> final_pos, std::vector<double> init_vel, std::vector<double> final_vel)
{
     init_vel.resize(_dof, 0.0);
     final_vel.resize(_dof, 0.0);

     // Ax=B
     Eigen::MatrixXd A = Eigen::MatrixXd::Zero(4, 4);     // matrix having coeff of the constants in cubic eqn.
     Eigen::MatrixXd A_INV = Eigen::MatrixXd::Zero(4, 4); // inveresee of the above matrix.
     Eigen::VectorXd x = Eigen::VectorXd::Zero(4);        // the inital and final condition vector having inital and final position and velocity.
     Eigen::VectorXd B = Eigen::VectorXd::Zero(4);        // the vector having the constant to be found out.

     //* filling the matrix having coeff of cubic eqn.
     A(0, 0) = 1.0;
     A(1, 1) = 1.0;
     A(2, 0) = 1.0;
     A(2, 1) = _finalTime;
     A(2, 2) = pow(_finalTime, 2);
     A(2, 3) = pow(_finalTime, 3);
     A(3, 1) = 1.0;
     A(3, 2) = 2.0 * (_finalTime);
     A(3, 3) = 3.0 * pow(_finalTime, 2);

     //*calculating inverse of the above matrix */
     A_INV = A.inverse();

     //* filling the "B" vector with the values of the different joints one by one and finding the coeff for all jonits and pushing them to the "_finalCoeffMat".

     for (size_t i = 0; i < _dof; i++)
     {
          std::vector<double> result;
          result.resize(x.size(), 0.0);

          B[0] = init_pos[i];
          B[1] = init_vel[i];
          B[2] = final_pos[i];
          B[3] = final_vel[i];

          x = A_INV * B; // * calculating the coefficients of cubic polynomial for all joints

          // * filling the values in "x" into another vector that will be pushed to the "_finalCoeffMat."

          for (size_t i = 0; i < x.size(); i++)
          {
               result[i] = x[i];
          }

          // printVec(result);

          _finalConstMat.emplace_back(result);

     } //! After this loop ends we'll have constants for all the joints in a matrix called "_finalCoeffMat".

     generatePathAndVel(_finalConstMat, _timeStep);
}

/**
 * @brief generates the total path and velocities for the joints from inital to final position.
 *
 * @param totalCoeffMat the coefficient matrix having coeff of cubic poly for respective joints.
 * @param linSpacedTime vector of equally spaced time intervals.
 */
void Cubic::generatePathAndVel(std::vector<std::vector<double>> totalCoeffMat, Eigen::VectorXd linSpacedTime)
{
     double t;
     std::vector<double> jointPosVec;
     std::vector<double> jointVelVec;

     for (size_t i = 0; i < linSpacedTime.size(); i++)
     {
          t = linSpacedTime[i]; // time goes from 0 to _finalTime.
          double posResult;
          double velResult;

          for (auto ele : totalCoeffMat)
          {
               // using the coefficients calculated above to generate path and velocities.
               posResult = (ele[0] * 1) + (ele[1] * t) + (ele[2] * t * t) + (ele[3] * t * t * t);

               velResult = (ele[0] * 0) + (ele[1] * 1) + (ele[2] * 2 * t) + (ele[3] * 3 * t * t);

               jointPosVec.emplace_back(posResult);
               jointVelVec.emplace_back(velResult);
          }

          // * print the vectors here to observe the values.
          // printVec(jointVelVec);

          _finalPath.emplace_back(jointPosVec);
          _finalVel.emplace_back(jointVelVec);

          jointPosVec.clear();
          jointVelVec.clear();
     }
     // * print the matrices here to observe the values.
     // printMat(_finalPath);
}

/**
 * @brief Destroy the Cubic::Cubic object
 *
 */
Cubic::~Cubic()
{
     std::cout << "SAB KHATAM KARDIA BHAI :/ "
               << "\n";
}

// ! HELPER FUNCTION TO PRINT THE VECTORS AND MATRICES. WILL BE REMOVE FROM HERE LATER. :)
void Cubic::printVec(std::vector<double> input)
{
     for (size_t i = 0; i < input.size(); i++)
     {
          std::cout << input[i] << " ";
     }
     std::cout << "\n\n";
}

void Cubic::printMat(std::vector<std::vector<double>> input)
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
