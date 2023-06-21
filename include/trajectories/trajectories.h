#ifndef B0C02644_559C_4143_B375_7892F742EB44
#define B0C02644_559C_4143_B375_7892F742EB44

#include <vector>

class Trajectories
{

public:
     virtual void findCoeff(std::vector<double> init_pos, std::vector<double> final_pos, std::vector<double> waypoint={}, std::vector<double> init_vel={}, std::vector<double> final_vel={}, std::vector<double> init_accel={}, std::vector<double> final_accel={}) = 0;
     virtual std::vector<std::vector<double>> &getPath() = 0;
     virtual std::vector<std::vector<double>> &getVel() = 0;
     // virtual std::vector<std::vector<double>> &acceleration() = 0;
};

#endif /* B0C02644_559C_4143_B375_7892F742EB44 */
