#ifndef BA1E3DEC_F93B_4A62_B8C8_A9CBD1093CA0
#define BA1E3DEC_F93B_4A62_B8C8_A9CBD1093CA0

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <vector>

class CubicViaPoint
{

public:
     CubicViaPoint(int dof, int viaptTime, int finalTime, int waypoints);

     void calcCoeffs(std::vector<double> init_pos, std::vector<double> viaPoint, std::vector<double> final_pos, std::vector<double> init_vel, std::vector<double> final_vel);

     void generatePathAndVel(std::vector<std::vector<double>> totalCoeffMat, Eigen::VectorXd linSpacedTime);

     void printVec(std::vector<double> input);
     void printMat(std::vector<std::vector<double>> input);

     std::vector<std::vector<double>> &getPath()
     {
          return std::ref(_finalPath);
     }
     std::vector<std::vector<double>> &getVel() { return std::ref(_finalVel); }

     ~CubicViaPoint();

private:
     int _dof;
     int _waypts;
     double _finalTime;
     double _viaPtTime;

     std::vector<std::vector<double>> _finalPath;
     std::vector<std::vector<double>> _finalVel;
     std::vector<std::vector<double>> _finalConstMat;

     // std::vector<double> _initial_pos;
     // std::vector<double> _final_pos;
     // std::vector<double> _inital_vel;
     // std::vector<double> _final_vel;

     Eigen::VectorXd _timeStep;
};

#endif /* BA1E3DEC_F93B_4A62_B8C8_A9CBD1093CA0 */
