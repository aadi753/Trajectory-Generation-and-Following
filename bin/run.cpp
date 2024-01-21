#include<Scurve.h>
#include<chrono>
int main ( ) {

     Scurve sc ( 1 );
     auto start = std::chrono::high_resolution_clock::now ( );
     // sc.calcCoeff ( { 10 } , 5 , 10 , 30 , { 1 } , { 0 } );
     sc.calcCoeff ( { 90 } ,20 , 20 , 20 , { 0 } , { 0 } );
     auto end = std::chrono::high_resolution_clock::now ( );
     auto dur = std::chrono::duration_cast<std::chrono::microseconds>( end - start );
     std::cout << dur.count ( ) << "\n";
     return 0;

     }