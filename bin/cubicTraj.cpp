#include<cubic.h>
#include<quintic.h>
#include<memory>
#include<chrono>

int main(){
     std::unique_ptr<Quintic> q = std::make_unique<Quintic>(6, 10, 300);

     std::vector<double> fPos{0.5, 0.3, 0.6, 0.7, 0.8, 0.4};
     std::vector<double> iPos{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
     std::vector<double> ivel{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
     std::vector<double> fvel{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
     std::vector<double> iacc{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
     std::vector<double> facc{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

     auto start = std::chrono::high_resolution_clock::now();
     q->calcCoeffs(iPos, fPos, ivel, fvel,iacc,facc);
     auto end = std::chrono::high_resolution_clock::now();
     auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
     std::cout << "TIME TAKEN IS: "<<duration.count()<<"ms "<< std::endl;
}