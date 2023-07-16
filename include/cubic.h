#ifndef ADA958D4_E490_42C2_B895_E8BF9AB6FF71
#define ADA958D4_E490_42C2_B895_E8BF9AB6FF71

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <vector>
#include <trajectories/trajectories.h>

class Cubic : public Trajectories
{

public:
     // Constructor

     /**
      * @brief Construct a new Cubic::Cubic object
      *
      * @param dof no. of joints for which trajectory is needed.
      * @param finalTime total time of trajectory.
      * @param waypoints no. of waypoints to be generated from inital to final point.
      */
     Cubic(int dof, int finalTime, int waypoints);

     // function to calculate the coefficients of the cubic polynomial
     /**
      * @brief calculates the coefficients of the cubic polynomial that are to be used for generating the path and the velocities.
      *
      * @param init_pos vector of intial joint positions.
      * @param final_pos vector of final joint positions.
      * @param init_vel vector of intial joint velocities.
      * @param final_vel vector of final joint velocities.
      */
     void calcCoeffs(std::vector<double> init_pos, std::vector<double> final_pos, std::vector<double> init_vel = {}, std::vector<double> final_vel = {});

     // function that generates the total path and velocities for the no. of DOF.
     /**
      * @brief generates the total path and velocities for the joints from inital to final position.
      *
      * @param totalCoeffMat the coefficient matrix having coeff of cubic poly for respective joints.
      * @param linSpacedTime vector of equally spaced time intervals.
      */
     void generatePathAndVel(std::vector<std::vector<double>> totalCoeffMat, Eigen::VectorXd linSpacedTime);

     // helper functions to print the vectors and matrices.
     void printVec(std::vector<double> input);
     void printMat(std::vector<std::vector<double>> input);

     // functions that will give the path and velocity matrices to be used further.
     std::vector<std::vector<double>> &getPath() { return std::ref(_finalPath); }
     std::vector<std::vector<double>> &getVel() { return std::ref(_finalVel); }
     std::vector<std::vector<double>> &getAccel() { return std::ref(_finalAccel); }

     // abstract class function.
     void findCoeff(std::vector<double> init_pos, std::vector<double> final_pos, std::vector<double> waypoint = {}, std::vector<double> init_vel = {}, std::vector<double> final_vel = {}, std::vector<double> init_accel = {}, std::vector<double> final_accel = {});

     ~Cubic();

private:
     int _dof;
     int _waypts;       // no. of waypoints
     double _finalTime; // total time of trajectory

     std::vector<std::vector<double>> _finalPath;
     std::vector<std::vector<double>> _finalVel;
     std::vector<std::vector<double>> _finalAccel;
     std::vector<std::vector<double>> _finalConstMat;

     Eigen::VectorXd _timeStep; // vector of euqally spaced time intervals.
};

#endif /* ADA958D4_E490_42C2_B895_E8BF9AB6FF71 */
