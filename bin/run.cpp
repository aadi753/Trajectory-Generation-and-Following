#include<Scurve.h>
#include<chrono>
#include <bits/stdc++.h>
#include<matplotlibcpp.h>

#define plt matplotlibcpp

int main ( ) {

     Scurve sc ( 6 );

     std::vector<double>pos , vel , acc , jerk;
     std::vector<double>pos1 , vel1 , acc1 , jerk1;
          // auto start = std::chrono::high_resolution_clock::now ( );
          // sc.calcCoeff ( { 10 } , 5 , 10 , 30 , { 1 } , { 0 } );
     sc.calcCoeffs ( { 0,0,0,0,0,0 } , { 10,-150,70,20,9,12 } , 30 , 30 , 30,{0},{0},false  );
     int joint = 1;
     for ( double i = 0; i < 51; i += 0.001 ) {
          bool status = sc.generatePathAndVel ( i , pos , vel , acc , jerk );
          if ( status == false )break;
          pos1.emplace_back ( pos [ joint ] );
          vel1.emplace_back ( vel [ joint ] );
          acc1.emplace_back ( acc [ joint ] );
          jerk1.emplace_back ( jerk [ joint ] );
          }
     plt::plot ( pos1 );
     plt::plot ( vel1 );
     plt::plot ( acc1 );
     plt::plot ( jerk1 );
     plt::show ( );
     // auto end = std::chrono::high_resolution_clock::now ( );
     // auto dur = std::chrono::duration_cast<std::chrono::microseconds>( end - start );
     // std::cout << dur.count ( ) << "\n";
     return 0;

     }