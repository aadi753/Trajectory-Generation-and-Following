#include <septic.h>

Septic::Septic(int dof, int finalTime, int waypoints)
{
     std::cout << "SEPTIC TRAJECTORY !! \n\n";
     _waypts = waypoints;
     _dof = dof;
     _finalTime = finalTime;
     Eigen::VectorXd tStep = Eigen::VectorXd::Zero(_waypts);

     tStep = tStep.LinSpaced(_waypts, 0, finalTime);
     _timeStep = tStep;
     // std::cout << "TIME STEP: " << tStep << " " << tStep.size() << "\n\n";
}

void Septic::calcCoeffs(std::vector<double> init_pos, std::vector<double> final_pos, std::vector<double> init_vel, std::vector<double> final_vel, std::vector<double> init_accel, std::vector<double> final_accel, std::vector<double> init_jerk, std::vector<double> final_jerk)
{
     init_vel.resize(_dof, 0.0);
     final_vel.resize(_dof, 0.0);

     init_accel.resize(_dof, 0.0);
     final_accel.resize(_dof, 0.0);

     init_jerk.resize(_dof, 0.0);
     final_jerk.resize(_dof, 0.0);

     //* Ax=B
     Eigen::MatrixXd A = Eigen::MatrixXd::Zero(8, 8);     // matrix having coeff of the constants in Septic eqn.
     Eigen::MatrixXd A_INV = Eigen::MatrixXd::Zero(8, 8); // inveresee of the above matrix.
     Eigen::VectorXd x = Eigen::VectorXd::Zero(8);        // the inital and final condition vector having inital and final position and velocity.
     Eigen::VectorXd B = Eigen::VectorXd::Zero(8);        // the vector having the constant to be found out.

     //* filling the matrix having coeff of Septic eqn.
     A(0, 0) = 1.0;
     A(1, 1) = 1.0;
     A(2, 2) = 2.0;
     A(3, 3) = 6.0;
     A(4, 0) = 1.0;
     A(4, 1) = _finalTime;
     A(4, 2) = pow(_finalTime, 2);
     A(4, 3) = pow(_finalTime, 3);
     A(4, 4) = pow(_finalTime, 4);
     A(4, 5) = pow(_finalTime, 5);
     A(4, 6) = pow(_finalTime, 6);
     A(4, 7) = pow(_finalTime, 7);
     A(5, 1) = 1.0;
     A(5, 2) = 2 * (_finalTime);
     A(5, 3) = 3 * pow(_finalTime, 2);
     A(5, 4) = 4 * pow(_finalTime, 3);
     A(5, 5) = 5 * pow(_finalTime, 4);
     A(5, 6) = 6 * pow(_finalTime, 5);
     A(5, 7) = 7 * pow(_finalTime, 6);
     A(6, 2) = 2.0;
     A(6, 3) = 6 * (_finalTime);
     A(6, 4) = 12 * pow(_finalTime, 2);
     A(6, 5) = 20 * pow(_finalTime, 3);
     A(6, 6) = 30 * pow(_finalTime, 4);
     A(6, 7) = 42 * pow(_finalTime, 5);
     A(7, 3) = 6.0;
     A(7, 4) = 24 * (_finalTime);
     A(7, 5) = 60 * pow(_finalTime, 2);
     A(7, 6) = 120 * pow(_finalTime, 3);
     A(7, 7) = 210 * pow(_finalTime, 4);

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
          B[3] = init_jerk[i];

          B[4] = final_pos[i];
          B[5] = final_vel[i];
          B[6] = final_accel[i];
          B[6] = final_jerk[i];

          x = A_INV * B;
          // std::cout << "X: " << x << " " << i << " " << "\n";

          // * filling the values in "x" into another vector that will be pushed to the "_finalCoeffMat."

          for (size_t i = 0; i < x.size(); i++)
          {
               result[i] = x[i];
          }

          // std::cout << "values of constants: "
          //           << "\n";

          // printVec(result);
          _finalConstMat.emplace_back(result);
          result.clear();

     } //! After this loop ends we'll have constants for all the joints in a matrix called "_finalCoeffMat".

     generatePathAndVel(_finalConstMat, _timeStep);
}

void Septic::generatePathAndVel(std::vector<std::vector<double>> totalCoeffMat, Eigen::VectorXd linSpacedTime)
{
     double t;
     std::vector<double> jointPosVec;
     std::vector<double> jointVelVec;
     std::vector<double> jointAccelVec;
     std::vector<double> jointJerkVec;

     for (size_t i = 0; i < linSpacedTime.size(); i++)
     {
          // std::cout << "index: " << i << std::endl;
          t = linSpacedTime[i];
          double posResult;
          double velResult;
          double accelResult;
          double jerkResult;

          for (auto ele : totalCoeffMat)
          {
               posResult = (ele[0] * 1) + (ele[1] * t) + (ele[2] * t * t) + (ele[3] * t * t * t) + (ele[4] * t * t * t * t) + (ele[5] * t * t * t * t * t) + (ele[6] * t * t * t * t * t * t) + (ele[7] * t * t * t * t * t * t * t);

               velResult = (ele[0] * 0) + (ele[1] * 1) + (ele[2] * 2 * t) + (ele[3] * 3 * t * t) + (ele[4] * 4 * t * t * t) + (ele[5] * 5 * t * t * t * t) + (ele[6] * 6 * t * t * t * t * t) + (ele[7] * 7 * t * t * t * t * t * t);

               accelResult = (ele[0] * 0) + (ele[1] * 0) + (ele[2] * 2) + (ele[3] * 6 * t) + (ele[4] * 12 * t * t) + (ele[5] * 20 * t * t * t) + (ele[6] * 30 * t * t * t * t) + (ele[7] * 42 * t * t * t * t * t);

               jerkResult = (ele[0] * 0) + (ele[1] * 0) + (ele[2] * 0) + (ele[3] * 6) + (ele[4] * 24 * t) + (ele[5] * 60 * t * t) + (ele[6] * 120 * t * t * t) + (ele[7] * 210 * t * t * t * t);

               jointPosVec.emplace_back(posResult);
               jointVelVec.emplace_back(velResult);
               jointAccelVec.emplace_back(accelResult);
               jointJerkVec.emplace_back(jerkResult);
          }

          // printVec(jointPosVec);

          _finalPath.emplace_back(jointPosVec);
          _finalVel.emplace_back(jointVelVec);
          _finalAccel.emplace_back(jointAccelVec);
          _finalJerk.emplace_back(jointJerkVec);

          jointPosVec.clear();
          jointVelVec.clear();
          jointAccelVec.clear();
          jointJerkVec.clear();
     }
     // printMat(_finalPath);
}

Septic::~Septic()
{
     // std::cout << "SAB KHATAM KARDIA BHAI :/ "
     //           << "\n";
}

void Septic::findCoeff(std::vector<double> init_pos, std::vector<double> final_pos, std::vector<double> waypoint, std::vector<double> init_vel, std::vector<double> final_vel, std::vector<double> init_accel, std::vector<double> final_accel)
{
     Septic::calcCoeffs(init_pos, final_pos, init_vel, final_vel, init_accel, final_accel);
}

// ! HELPER FUNCTION TO PRINT THE VECTORS AND MATRICES. WILL BE REMOVED LATER :)
void Septic::printVec(std::vector<double> input)
{
     for (size_t i = 0; i < input.size(); i++)
     {
          std::cout << input[i] << " ";
     }
     std::cout << "\n\n";
}

void Septic::printMat(std::vector<std::vector<double>> input)
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
