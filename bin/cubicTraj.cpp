#include<cubic.h>
#include<memory>
#include<chrono>

int main(){
     std::unique_ptr<Cubic> cube = std::make_unique<Cubic>(6, 10,500);

     std::vector<double> fPos{0.5, 0.3, 0.6, 0.7, 0.8, 0.4};
     std::vector<double> iPos{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
     std::vector<double> ivel{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
     std::vector<double> fvel{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

     auto start = std::chrono::high_resolution_clock::now();
     cube->calcCoeffs(iPos, fPos, ivel, fvel);
     auto end = std::chrono::high_resolution_clock::now();
     auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
     std::cout << "TIME TAKEN IS: "<<duration.count()<<"ms "<< std::endl;
}