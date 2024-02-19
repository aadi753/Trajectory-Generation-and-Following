#ifndef DA23E969_70DB_4BC8_8296_18614F724C6B
#define DA23E969_70DB_4BC8_8296_18614F724C6B

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <vector>
#include <trajectories.h>

class ParabolicBlend : public Trajectories
{

public:
     // Constructor
     /**
      * @brief Construct a new Parabolic Blend:: Parabolic Blend object
      *
      * @param dof number of joints for which trajectory is to be generated.
      * @param finalTime total time of trajectory.
      * @param waypoints number of points to be generated in the trajectory.
      */
     ParabolicBlend(int dof, int finalTime, int waypoints = 100);

     // Function to calculate coefficients for path and velocity generation.
     /**
      * @brief calculates the blend time for the parabolic blend
      *
      * @param init_pos vector of intial positions of joints.
      * @param final_pos vector of final positions of joints.
      * @param blendVel velocity at blend point.
      * @param blendAccel acceleration at blend point.
      *
      * @note keep the values of blendVel and blendAccel equal, by default they are set to 0.25 don't change it if not sure how it affects the trajectory.
      */
     void calcCoeffs(std::vector<double> init_pos, std::vector<double> final_pos, double blendVel = 0.25, double blendAccel = 0.25);

     // Function to generate the final path ,velocities and accelerations
     /**
      * @brief generates the final path ,velocities and accelerations
      *
      * @param init_final_posMat matrix having intial and final positions of the joints.
      * @param linSpacedTime vector of equally spaced time intervals.
      */
     void generatePathAndVel(std::vector<std::vector<double>> init_final_posMat, Eigen::VectorXd linSpacedTime);

     // Helper function to print the matrices and vectors.
     void printVec(std::vector<double> input);
     void printMat(std::vector<std::vector<double>> input);

     std::vector<std::vector<double>> &getPath() { return std::ref(_finalPath); }
     std::vector<std::vector<double>> &getVel() { return std::ref(_finalVel); }
     std::vector<std::vector<double>> &getAccel() { return std::ref(_finalAccel); }

     // abstract class function.
     void findCoeff(std::vector<double> init_pos, std::vector<double> final_pos, std::vector<double> waypoint = {}, std::vector<double> init_vel = {}, std::vector<double> final_vel = {}, std::vector<double> init_accel = {}, std::vector<double> final_accel = {});

     ~ParabolicBlend();

private:
     int _dof;
     int _waypts;
     double _finalTime;

     double _blendTime;  // time after which the linear portion starts
     double _blendVel;   // velocity at the blend point
     double _blendAccel; // acceleration at the blend point

     std::vector<std::vector<double>> _finalPath;
     std::vector<std::vector<double>> _finalVel;
     std::vector<std::vector<double>> _finalAccel;
     std::vector<std::vector<double>> _finalCoeffMat;

     Eigen::VectorXd _timeStep; // vector of equally spaced time intervals
};

#endif /* DA23E969_70DB_4BC8_8296_18614F724C6B */
