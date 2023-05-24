#include <parabolicBlend.h>
// TODO . resize the vectors in constructor.
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

void ParabolicBlend::calcCoeffs(std::vector<double> init_pos, std::vector<double> blendVel, std::vector<double> blendAccel)
{
     _blendVel = blendVel;
     _blendAccel = blendAccel;
     // TODO add a size check with dof for the 2 vectors.
     std::vector<double> results;
     for (size_t i = 0; i < _dof; i++)
     {
          _blendTime[i] = blendVel[i] / blendAccel[i];

          results.emplace_back(_blendTime[i]);
          results.emplace_back(init_pos[i]);
          results.emplace_back(blendVel[i]);
          results.emplace_back(blendAccel[i]);

          _finalCoeffMat.emplace_back(results);
          results.clear();
     }

     generatePathAndVel(_finalCoeffMat, _timeStep);
}

void ParabolicBlend::generatePathAndVel(std::vector<std::vector<double>> totalCoeffMat, Eigen::VectorXd linSpacedTime)
{
     double t;
     std::vector<double> posVec;
     std::vector<double> velVec;
     std::vector<double> accelVec;

     for (size_t i = 0; i < linSpacedTime.size(); i++)
     {
          t = linSpacedTime[i];
          double pos, vel, accel;

          for (auto ele : totalCoeffMat)
          {
               if (t < ele[0])
               {
                    pos = ele[1] + ((1 / 2) * ele[3] * t * t);
                    vel = ele[3] * t;
                    accel = ele[3];
                    posVec.emplace_back(pos);
                    velVec.emplace_back(vel);
                    accelVec.emplace_back(accel);
               }

               else if (t > ele[0] && t < _finalTime - ele[0])
               {
                    pos = ((1 / 2) * ele[3] * pow(ele[0], 2)) + (ele[2] * (_finalTime - ele[0]));
                    vel = ele[2];
                    accel = 0;
                    posVec.emplace_back(pos);
                    velVec.emplace_back(vel);
                    accelVec.emplace_back(accel);
               }
               else
               {
                    pos = ((1 / 2) * ele[3] * pow(ele[0], 2)) + (ele[2] * (_finalTime - 2 * ele[0])) - ((1 / 2) * ele[3] * pow((t - (_finalTime - ele[0])), 2));
                    vel = ele[2] - ele[3] * (t - (_finalTime - ele[0]));
                    accel = -(ele[0]);
                    posVec.emplace_back(pos);
                    velVec.emplace_back(vel);
                    accelVec.emplace_back(accel);
               }
          }

          // print the vectors here to see the values.

          _finalPath.emplace_back(posVec);
          _finalVel.emplace_back(velVec);
          _finalAccel.emplace_back(accelVec);

          posVec.clear();
          velVec.clear();
          accelVec.clear();
     }

     // print the matrices here to see the values.
}

ParabolicBlend::~ParabolicBlend()
{
     std::cout << "SAB KHATAM KARDIA BHAI :/ "
               << "\n";
}