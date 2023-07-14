#include <cubic.h>
#include <cubic_via_point.h>
#include <quintic.h>
#include <parabolicBlend.h>
#include <septic.h>
#include <memory>
#include <chrono>
#include <trajectories/trajectories.h>

enum trajectories_
{
     cubic,
     cubicViapoint,
     parabolicBlend,
     quintic,
     septic
};

std::unique_ptr<Trajectories> traj;

void printMat(std::vector<std::vector<double>> &input)
{
     for (auto &ele : input)
     {
          for (auto &el : ele)
          {
               std::cout << el << " ";
          }
          std::cout << "\n";
     }
}

int main()
{
     int selected_traj = trajectories_::cubicViapoint;
     switch (selected_traj)
     {
     case trajectories_::cubic:
          traj = std::make_unique<Cubic>(6, 5, 200);
          break;
     case trajectories_::cubicViapoint:
          traj = std::make_unique<CubicViaPoint>(6, 2,5, 200);
          break;
     case trajectories_::parabolicBlend:
          traj = std::make_unique<ParabolicBlend>(6, 5, 200);
          break;
     case trajectories_::quintic:
          traj = std::make_unique<Quintic>(6, 5, 200);
          break;
     case trajectories_::septic:
          traj = std::make_unique<Septic>(6, 5, 200);
          break;
     default:
          std::cout << "PLEASE CHOOSE A VALID TRAJECTORY TYPE FROM THE ABOVE LIST!!" << std::endl;
          break;
     }

     std::vector<double> fPos{0.5, 0.3, 0.9, 0.7, 8, 0.4};

     // std::vector<double> fPos{0.9, 0.6, 0.6, 1.7, 0.8, 0.7};

     std::vector<double> viapt{0.25, 0.15, 0.25, 0.35, 4, 0.2};

     std::vector<double> iPos{0.0, 0.0, 0.0, 0.0, 1, 0.0};

     std::vector<double> ivel{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

     std::vector<double> fvel{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

     std::vector<double> iacc{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

     std::vector<double> facc{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

     std::vector<double> ijerk{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

     std::vector<double> fjerk{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

     auto start = std::chrono::high_resolution_clock::now();

     // q->calcCoeffs(iPos, fPos, ivel, fvel, iacc, facc);

     traj->findCoeff(iPos, fPos, viapt);
     // printMat(traj->getPath());
     std::cout << "\n\n\n";
     // printMat(traj->getVel());

     auto end = std::chrono::high_resolution_clock::now();
     auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
     std::cout << "TIME TAKEN IS: " << duration.count() << "ms " << std::endl;
}