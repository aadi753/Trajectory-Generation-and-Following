/**
 * @file parabolicBlend.cpp
 * @author Aditya Singh (aditya.in753@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-05-31
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <parabolicBlend.h>

/**
 * @brief Construct a new Parabolic Blend:: Parabolic Blend object
 *
 * @param dof number of joints for which trajectory is to be generated.
 * @param finalTime total time of trajectory.
 * @param waypoints number of points to be generated in the trajectory.
 */
ParabolicBlend::ParabolicBlend(int dof, int finalTime, int waypoints)
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
 * @brief calculates the blend time for the parabolic blend
 *
 * @param init_pos vector of intial positions of joints.
 * @param final_pos vector of final positions of joints.
 * @param blendVel velocity at blend point.
 * @param blendAccel acceleration at blend point.
 *
 * @note keep the values of blendVel and blendAccel equal, by default they are set to 0.25 don't change it if not sure how it affects the trajectory.
 */
void ParabolicBlend::calcCoeffs(std::vector<double> init_pos, std::vector<double> final_pos, double blendVel , double blendAccel)
{
     init_pos.resize(_dof, 0.0);
     final_pos.resize(_dof, 0.0);

     _blendVel = blendVel;
     _blendAccel = blendAccel;

     _blendTime = _blendVel / _blendAccel;

     std::vector<double> results;
     for (size_t i = 0; i < _dof; i++)
     {
          results.emplace_back(init_pos[i]);
          results.emplace_back(final_pos[i]);

          _finalCoeffMat.emplace_back(results);
          results.clear();
     }

     generatePathAndVel(_finalCoeffMat, _timeStep);
}

/**
 * @brief generates the final path ,velocities and accelerations
 *
 * @param init_final_posMat matrix having intial and final positions of the joints.
 * @param linSpacedTime vector of equally spaced time intervals.
 */
void ParabolicBlend::generatePathAndVel(std::vector<std::vector<double>> init_final_posMat, Eigen::VectorXd linSpacedTime)
{
     double t;

     std::vector<double> posVec;
     std::vector<double> velVec;
     std::vector<double> accelVec;

     for (size_t i = 0; i < linSpacedTime.size(); i++)
     {
          t = linSpacedTime[i];
          double s, s_dot, s_ddot;

          for (auto ele : init_final_posMat)
          {
               if (t < _blendTime) // 1st blend portion
               {
                    s = ((0.5) * _blendAccel * t * t);

                    s_dot = _blendAccel * t;

                    s_ddot = _blendAccel;
               }

               else if (t > _blendTime && t < (_finalTime - _blendTime)) // linear portion
               {
                    s = ((0.5) * (_blendAccel * (_blendTime * _blendTime))) + (_blendVel * (t - _blendTime));

                    s_dot = ele[2];

                    s_ddot = 0;
               }
               else // final blend portion
               {

                    s = ((0.5) * _blendAccel * pow(_blendTime, 2)) + (_blendVel * (_finalTime - (2 * _blendTime))) + (_blendVel * (t - (_finalTime - _blendTime))) - (((0.5) * _blendAccel) * pow((t - (_finalTime - _blendTime)), 2));

                    s_dot = ele[2] - _blendAccel * (t - (_finalTime - _blendTime));

                    s_ddot = -(_blendAccel);
               }
               // calculating the final pos,vel,accel values.
               double pos = ele[0] + (ele[1] - ele[0]) * s;
               double vel = (ele[1] - ele[0]) * s_dot;
               double accel = (ele[1] - ele[0]) * s_ddot;

               posVec.emplace_back(pos);
               velVec.emplace_back(vel);
               accelVec.emplace_back(accel);
          }

          //* print the vectors here to see the values.
          // std::cout << posVec[2] << "\n";

          _finalPath.emplace_back(posVec);
          _finalVel.emplace_back(velVec);
          _finalAccel.emplace_back(accelVec);

          posVec.clear();
          velVec.clear();
          accelVec.clear();
     }

     //* print the matrices here to see the values.
     printMat(_finalPath);
}

/**
 * @brief Destroy the Parabolic Blend:: Parabolic Blend object
 *
 */
ParabolicBlend::~ParabolicBlend()
{
     std::cout << "SAB KHATAM KARDIA BHAI :/ "
               << "\n";
}

void ParabolicBlend::printVec(std::vector<double> input)
{
     for (size_t i = 0; i < input.size(); i++)
     {
          std::cout << input[i] << " ";
     }
     std::cout << "\n";
}

void ParabolicBlend::printMat(std::vector<std::vector<double>> input)
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