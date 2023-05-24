#ifndef DA23E969_70DB_4BC8_8296_18614F724C6B
#define DA23E969_70DB_4BC8_8296_18614F724C6B

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <vector>

class ParabolicBlend
{

public:
     ParabolicBlend(int dof, int finalTime, int waypoints);

     void calcCoeffs(std::vector<double> init_pos,std::vector<double> blendVel, std::vector<double> blendAccel);

     void generatePathAndVel(std::vector<std::vector<double>> totalCoeffMat, Eigen::VectorXd linSpacedTime);

     void printVec(std::vector<double> input);
     void printMat(std::vector<std::vector<double>> input);

     std::vector<std::vector<double>> &getPath()
     {
          return std::ref(_finalPath);
     }
     std::vector<std::vector<double>> &getVel() { return std::ref(_finalVel); }

     ~ParabolicBlend();

private:
     int _dof;
     int _waypts;
     double _finalTime;

     std::vector<std::vector<double>> _finalPath;
     std::vector<std::vector<double>> _finalVel;
     std::vector<std::vector<double>> _finalAccel;
     std::vector<std::vector<double>> _finalCoeffMat;

     std::vector<double> _blendTime;
     std::vector<double> _blendVel;
     std::vector<double> _blendAccel;

     // std::vector<double> _inital_vel;
     // std::vector<double> _final_vel;

     Eigen::VectorXd _timeStep;
};

#endif /* DA23E969_70DB_4BC8_8296_18614F724C6B */
