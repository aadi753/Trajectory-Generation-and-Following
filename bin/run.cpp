#include<Scurve.h>
#include<chrono>
#include <bits/stdc++.h>
#include<matplotlibcpp.h>

#define plt matplotlibcpp

int main ( ) {

     Scurve sc ( 1 );

     std::vector<double>pos , vel , acc , jerk;
     std::vector<double>pos1 , vel1 , acc1 , jerk1;
          // auto start = std::chrono::high_resolution_clock::now ( );
          // sc.calcCoeff ( { 10 } , 5 , 10 , 30 , { 1 } , { 0 } );
     sc.calcCoeff ( { 0 } , { 90 } , 60 , 30 , 50 , { 0.2 } , { 0 } );

     for ( double i = 0; i < 4.1; i += 0.01 ) {
         bool status= sc.generatePathAndVel ( i , pos , vel , acc , jerk );
         if ( status == false )break;
         pos1.emplace_back ( pos [ 0 ] );
         vel1.emplace_back ( vel [ 0 ] );
          acc1.emplace_back ( acc [ 0 ] );
          jerk1.emplace_back ( jerk [ 0 ] );
          }
     plt::plot ( pos1 );
     plt::plot ( vel1 );
     plt::plot ( acc1 );
     plt::plot ( jerk1 );
     plt::show ( );
     // auto end = std::chrono::high_resolution_clock::now ( );
     // auto dur = std::chrono::duration_cast<std::chrono::microseconds>( end - start );
     return 0;

     }