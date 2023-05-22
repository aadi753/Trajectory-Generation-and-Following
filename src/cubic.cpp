#include <cubic.h>

Cubic::Cubic(int finalTime)
{
     Eigen::VectorXd tStep;
     tStep.LinSpaced(200, 0, finalTime);
}

void Cubic::calcCoeffs(std::vector<double> init_pos, std::vector<double>final_pos,std::vector<double>init_vel,std::vector<double> final_vel){
     
}