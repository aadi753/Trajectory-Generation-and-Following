#ifndef ADA958D4_E490_42C2_B895_E8BF9AB6FF71
#define ADA958D4_E490_42C2_B895_E8BF9AB6FF71

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <vector>

class Cubic
{
private:
     // const int _dof;
     // const double timeStep;

     std::vector<std::vector<double>> _finalPath;
     std::vector<std::vector<double>> _finalVel;
     std::vector<std::vector<double>> _finalCoeffMat;

     std::vector<double> _initial_pos;
     std::vector<double> _final_pos;
     std::vector<double> _inital_vel;
     std::vector<double> _final_vel;

public:
     Cubic(int finalTime);
     void calcCoeffs(std::vector<double> init_pos, std::vector<double> final_pos, std::vector<double> init_vel, std::vector<double> final_vel);
     void generatePath(std::vector<std::vector<double>> totalCoeffMat);
     std::vector<std::vector<double>> &getPath() { return std::ref(_finalPath); }
     std::vector<std::vector<double>> &getVel() { return std::ref(_finalVel); }
     ~Cubic();
};

#endif /* ADA958D4_E490_42C2_B895_E8BF9AB6FF71 */
