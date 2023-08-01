#ifndef B0B7C493_9822_413C_9647_41A05E2EF5CD
#define B0B7C493_9822_413C_9647_41A05E2EF5CD

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <vector>
#include <trajectories.h>

class QuinticMultiViapt : public Trajectories
{

public:
     // Constructor that takes 4 input parameters
     /**
      * @brief Construct a new Cubic Via Point:: Cubic Via Point object
      *  takes in the following as input and also generates equally spaced time points.

     * @param dof  number of joints for which trajectory is needed
     * @param viaptTime  time to reach the viaPoint.
     * @param finalTime  total time of trajectory
     * @param waypoints  no. of waypoints in the trajectory
     */
     QuinticMultiViapt(int dof, int viaptTime, int finalTime, int waypoints, std::vector<std::vector<double>> waypointList);

     // function to calculate the coefficients of the cubic poly.
     /**
      * @brief used to calculate the coefficients of the cubic polynomial that are to be used for generating the positions and velocities.
      *
      * @param init_pos a vector of inital joint positions.
      * @param viaPoint a vector of joint positions at the viaPoint.
      * @param final_pos a vector of final joint positions.
      * @param init_vel  a vector of inital joint velocities.
      * @param final_vel a vector of final joint velocities.
      */
     void calcCoeffs(std::vector<double> init_pos, std::vector<double> viaPoint, std::vector<double> final_pos, std::vector<double> init_vel = {}, std::vector<double> final_vel = {}, bool lastSegment = false, std::vector<double> init_accel = {},std::vector<double>final_accel={});

     // function that generates the positions,velocities.
     /**
      * @brief generates the total path and the velocities for the no. of DOF.
      *
      * @param totalCoeffMat matrix of coefficients of the cubic poly for all DOF. size should be 8xDOF(m x n).
      * @param linSpacedTime vector of equally spaced time intervals.
      */
     void generatePathAndVel(std::vector<std::vector<double>> totalCoeffMat, Eigen::VectorXd linSpacedTime, bool lastSegment);

     void blendWaypoints(std::vector<std::vector<double>> wayptVector); // takes the vector of waypoints and calls other functions to generate a blended trajectory through all the pts

     // helper functions to print the vectors and matrices.
     void printVec(std::vector<double> input);
     void printMat(std::vector<std::vector<double>> input);

     // functions to get the path and velocity matrices that are to be used.
     std::vector<std::vector<double>> &getPath() { return std::ref(_finalPath); }
     std::vector<std::vector<double>> &getVel() { return std::ref(_finalVel); }
     std::vector<std::vector<double>> &getAccel() { return std::ref(_finalAccel); }

     // abstarct class function
     void findCoeff(std::vector<double> init_pos, std::vector<double> final_pos, std::vector<double> waypoint = {}, std::vector<double> init_vel = {}, std::vector<double> final_vel = {}, std::vector<double> init_accel = {}, std::vector<double> final_accel = {});

     ~QuinticMultiViapt();

private:
     int _dof;    // no. of joints
     int _waypts; // no. of waypts
     double _finalTime;
     double _viaPtTime; // time to reach for viaPoint

     std::vector<std::vector<double>> _waypointList;
     std::vector<std::vector<double>> _totalPath;
     std::vector<std::vector<double>> _totalVel;

     std::vector<std::vector<double>> _finalPath;
     std::vector<std::vector<double>> _finalVel;
     std::vector<std::vector<double>> _finalAccel;
     std::vector<std::vector<double>> _finalConstMat;

     Eigen::VectorXd _timeStep; // equally spaced time intervals.
};

#endif /* B0B7C493_9822_413C_9647_41A05E2EF5CD */
