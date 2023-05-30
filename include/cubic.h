#ifndef ADA958D4_E490_42C2_B895_E8BF9AB6FF71
#define ADA958D4_E490_42C2_B895_E8BF9AB6FF71

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <vector>

class Cubic
{

public:
     // Constructor 
     Cubic(int dof, int finalTime, int waypoints);

     // function to calculate the coefficients of the cubic polynomial
     void calcCoeffs(std::vector<double> init_pos, std::vector<double> final_pos, std::vector<double> init_vel, std::vector<double> final_vel);

     // function that generates the total path and velocities for the no. of DOF.
     void generatePathAndVel(std::vector<std::vector<double>> totalCoeffMat, Eigen::VectorXd linSpacedTime);
     
     // helper functions to print the vectors and matrices.
     void printVec(std::vector<double> input);
     void printMat(std::vector<std::vector<double>> input);

     // functions that will give the path and velocity matrices to be used further.
     std::vector<std::vector<double>> &getPath()
     {
          return std::ref(_finalPath);
     }
     std::vector<std::vector<double>> &getVel() { return std::ref(_finalVel); }

     ~Cubic();

private:
     int _dof;
     int _waypts; // no. of waypoints
     double _finalTime; // total time of trajectory

     std::vector<std::vector<double>> _finalPath;
     std::vector<std::vector<double>> _finalVel;
     std::vector<std::vector<double>> _finalConstMat;

     Eigen::VectorXd _timeStep; // vector of euqally spaced time intervals.
};

#endif /* ADA958D4_E490_42C2_B895_E8BF9AB6FF71 */
