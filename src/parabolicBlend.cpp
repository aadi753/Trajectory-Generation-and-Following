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

     _blendTime.resize(_dof, 0);
}

void ParabolicBlend::calcCoeffs(std::vector<double> init_pos, std::vector<double> final_pos, std::vector<double> blendVel, std::vector<double> blendAccel)
{
     _blendVel = blendVel;
     _blendAccel = blendAccel;

     // TODO add a size check with dof for the 2 vectors.

     std::vector<double> results;
     for (size_t i = 0; i < _dof; i++)
     {
          _blendTime[i] = _blendVel[i] / _blendAccel[i];

          results.emplace_back(_blendTime[i]);
          results.emplace_back(init_pos[i]);
          results.emplace_back(_blendVel[i]);
          results.emplace_back(_blendAccel[i]);
          results.emplace_back(final_pos[i]);

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
          double s, s_dot, s_ddot;

          for (auto ele : totalCoeffMat)
          {
               if (t < ele[0])
               {
                    s = ((0.5) * ele[3] * t * t);
                    
                    s_dot = ele[3] * t;
                    s_ddot = ele[3];

                    
               }

               else if (t > ele[0] && t < (_finalTime - ele[0]))
               {
                    s = ((0.5) * (ele[3] * (ele[0] * ele[0]))) + (ele[2] * (t - ele[0]));
                    s_dot = ele[2];
                    s_ddot = 0;
               }
               else
               {
               
                    s = ((0.5) * ele[3] * pow(ele[0], 2)) + (ele[2] * (_finalTime - (2 * ele[0]))) + (((0.5) * ele[3]) * pow((t - (_finalTime - ele[0])), 2));
                    s_dot = ele[2] - ele[3] * (t - (_finalTime - ele[0]));
                    s_ddot = -(ele[0]);
               }
               double pos = ele[1] + (ele[4] - ele[1]) * s;
               double vel = (ele[4] - ele[1]) * s_dot;
               double accel = (ele[4] - ele[1]) * s_ddot;

               posVec.emplace_back(pos);
               velVec.emplace_back(vel);
               accelVec.emplace_back(accel);
          }

          //* print the vectors here to see the values.

     
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