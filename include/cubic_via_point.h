#ifndef BA1E3DEC_F93B_4A62_B8C8_A9CBD1093CA0
#define BA1E3DEC_F93B_4A62_B8C8_A9CBD1093CA0

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <vector>

class CubicViaPoint
{

public:
     // Constructor that takes 4 input parameters
     CubicViaPoint(int dof, int viaptTime, int finalTime, int waypoints);
     
     // function to calculate the coefficients of the cubic poly.
     void calcCoeffs(std::vector<double> init_pos, std::vector<double> viaPoint, std::vector<double> final_pos, std::vector<double> init_vel, std::vector<double> final_vel);
     
     // function that generates the positions,velocities.
     void generatePathAndVel(std::vector<std::vector<double>> totalCoeffMat, Eigen::VectorXd linSpacedTime);
     
     // helper functions to print the vectors and matrices.
     void printVec(std::vector<double> input);
     void printMat(std::vector<std::vector<double>> input);

     // functions to get the path and velocity matrices that are to be used.
     std::vector<std::vector<double>> &getPath()
     {
          return std::ref(_finalPath);
     }
     std::vector<std::vector<double>> &getVel() { return std::ref(_finalVel); }

     ~CubicViaPoint();

private:
     int _dof; // no. of joints 
     int _waypts; // no. of waypts
     double _finalTime; 
     double _viaPtTime; // time to reach for viaPoint

     std::vector<std::vector<double>> _finalPath;
     std::vector<std::vector<double>> _finalVel;
     std::vector<std::vector<double>> _finalConstMat;


     Eigen::VectorXd _timeStep; // equally spaced time intervals.
};

#endif /* BA1E3DEC_F93B_4A62_B8C8_A9CBD1093CA0 */
