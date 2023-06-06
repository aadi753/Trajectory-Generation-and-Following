#include <cubic.h>
#include <cubic_via_point.h>
#include <quintic.h>
#include <parabolicBlend.h>
#include <septic.h>

#include <memory>
#include <chrono>

int main()
{
     std::unique_ptr<Cubic> q = std::make_unique<Cubic>(6, 8, 400);

     std::vector<double> fPos{0.5, 0.3, 0.9, 0.7, 8, 0.4};

     // std::vector<double> fPos{0.9, 0.6, 0.6, 1.7, 0.8, 0.7};

     std::vector<double> viapt{0.25, 0.15, 0.45, 0.35, 4, 0.2};

     std::vector<double> iPos{0.0, 0.0, 0.0, 0.0, 1, 0.0};

     std::vector<double> ivel{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

     std::vector<double> fvel{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

     std::vector<double> iacc{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

     std::vector<double> facc{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

     std::vector<double> ijerk{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

     std::vector<double> fjerk{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

     auto start = std::chrono::high_resolution_clock::now();
     // q->calcCoeffs(iPos, fPos,ivel,fvel,iacc,facc,ijerk,fjerk);

     q->calcCoeffs(iPos,fPos, ivel, fvel);

     auto end = std::chrono::high_resolution_clock::now();
     auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

     std::cout << "TIME TAKEN IS: " << duration.count() << "ms " << std::endl;
}