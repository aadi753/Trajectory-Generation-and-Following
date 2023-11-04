#include <cubic.h>
#include <cubic_via_point.h>
#include <cubicMultiViaPoint.h>
#include <quintic.h>
#include <parabolicBlend.h>
#include <quinticMulitViapt.h>
#include <septic.h>
#include <memory>
#include <chrono>
#include <trajectories/trajectories.h>
#include <matplotlibcpp.h>
#include <quintUpdated.h>
#include <septicUpdated.h>

#define plt matplotlibcpp
#define waypts 400

enum trajectories_
{
     cubic,
     cubicViapoint,
     cubicMultiViaPt,
     parabolicBlend,
     quintic,
     quinticMulipt,
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

void plotVecFromMat(std::vector<std::vector<double>> &inp)
{
     std::vector<double> printable;
     printable.resize(inp.size(), 0.0);
     for (size_t i = 0; i < inp.size(); i++)
     {
          printable[i] = inp[i][4];
     }
     plt::plot(printable);
     // plt::show();
}

void getVecFromMat(std::vector<double> &inpt)
{
     std::vector<double> printable;
     for (auto &ele : inpt)
     {
          printable.emplace_back(ele);
     }
     plt::plot(printable);
     plt::show();
}

int main()
{
     // 15,20,-148,-159,150,152
     
     // std::vector<std::vector<double>> _waypointList = {{0, 0, 0, 0, 0, 0}, {0, 60, 0, 0, 0, 0}, {0, 90, 0, 0, 0, 0}, {0, 150, 0, 0, 0, 0}, {0, 200, 0, 0, 0, 0}};
     std::vector<std::vector<double>> _waypointList = {{0, 0, 0, 0, 0, 0}, {40, 30, 30, 40, 20, 60}, {80, -10, -10,120 , 30, 70}, {0, 0, 0, 0, 0, 0}};
     // std::vector<std::vector<double>> _waypointList = {{0, 0, 0, 0, 0, 0}, {10, 30, 30, 40, 50, 60}, {50, 0, -10, 50, 60, 70}, {40, 40, -30, 50, 60, 70}, {50, 40, 20, 90, 27, 28}, {40, 30, 30, 40, 50, 60}, {80, -10, -10, 50, 60, 70}, {-30, 40, -30, 50, 60, 70}, {50, 10, 20, 90, 27, 28}};

     // std::vector<std::vector<double>> _waypointList = {{0, 0, 0, 0, 0, 0}, {10, 30, 30, 40, 50, 60}, {50, 0, -10, 50, 60, 70}, {40, 40, -30, 50, 60, 70}, {50, 40, 20, 90, 27, 28}, {40, 30, 30, 40, 50, 60}, {80, -10, -10, 50, 60, 70}, {-30, 40, -30, 50, 60, 70}, {50, 40, 20, 90, 27, 28}};

     // std::vector<std::vector<double>> _waypointList = {{0, 0, 0, 0, 0, 0}, {0, 40, 0, 0, 0, 0}, {0, 60, 0, 0, 0, 0}, {0, 90, 0, 0, 0, 0}, {0, 110, 0, 0, 0, 0}, {0, 50, 0, 0, 27, 28}};

     int selected_traj = trajectories_::quinticMulipt;
     switch (selected_traj)
     {
     case trajectories_::cubic:
          traj = std::make_unique<Cubic>(6, 5, waypts);
          break;
     case trajectories_::cubicViapoint:
          traj = std::make_unique<CubicViaPoint>(6, 2, 5, waypts);
          break;
     case trajectories_::parabolicBlend:
          traj = std::make_unique<ParabolicBlend>(6, 3, waypts);
          break;
     case trajectories_::quintic:
          traj = std::make_unique<Quintic>(6, 5, waypts);
          break;
     case trajectories_::septic:
          traj = std::make_unique<Septic>(6, 3, waypts);
          break;
     case trajectories_::cubicMultiViaPt:
          traj = std::make_unique<CubicMultiViaPoint>(6, 7, 10, 200, _waypointList);
          break;
     case trajectories_::quinticMulipt:
          traj = std::make_unique<QuinticMultiViapt>(6, 5, 10, 100, _waypointList);
          break;
     default:
          std::cout << "PLEASE CHOOSE A VALID TRAJECTORY TYPE FROM THE ABOVE LIST!!" << std::endl;
          break;
     }

     std::vector<double> fPos{1, 0.3, 0.9, 0.7, 20, 0.4};

     // std::vector<double> fPos{0.9, 0.6, 0.6, 1.7, 0.8, 0.7};

     std::vector<double> viapt{0.25, 0.15, 0.4, 0.35, 4, 0.2};

     std::vector<double> iPos{0.0, 0.0, 0.0, 0.0, 0, 0.0};

     std::vector<double> ivel{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

     std::vector<double> fvel{0.0, 20.0, 0.0, 0.0, 0.0, 0.0};

     std::vector<double> iacc{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

     std::vector<double> facc{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

     std::vector<double> ijerk{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

     std::vector<double> fjerk{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
     std::vector<std::vector<double>>{{0, 0, 0, 0, 0, 0}, {10, 20, 30, 40, 50, 60}, {20, 30, 40, 50, 60, 70}, {30, 40, 50, 60, 70, 80}, {123, 124, 125, 126, 127, 128}};

     // auto start = std::chrono::high_resolution_clock::now();
     // std::unique_ptr<SEPTIC> t = std::make_unique<SEPTIC>(6);
     // std::unique_ptr<quint> t1 = std::make_unique<quint>(6, 1);
     // t->calcCoeffs(iPos, fPos, 20, 20, 20);
     // t1->calcCoeffs(iPos, fPos, 20, 20);

     traj->findCoeff(iPos, fPos);
     plotVecFromMat(traj->getPath());
     plotVecFromMat(traj->getVel());
     plotVecFromMat(traj->getAccel());
     // // printMat(traj->getPath());
     // // std::cout << "\n\n\n";
     //// printMat(traj->getVel());
     plt::show();
     // auto end = std::chrono::high_resolution_clock::now();
     // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
     // std::cout << "TIME TAKEN IS: " << duration.count() << "ms " << std::endl;
}