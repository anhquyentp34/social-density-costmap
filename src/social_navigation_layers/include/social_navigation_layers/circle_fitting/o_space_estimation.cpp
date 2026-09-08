// This code is based on the algebraic circle fits
// It worked well
#include "mystuff.h"
#include "data.h"
#include "circle.h"
#include "Utilities.cpp"
#include "CircleFitByTaubin.cpp"
#include "CircleFitByPratt.cpp"
#include "CircleFitByKasa.cpp"
#include "CircleFitByHyper.cpp"
#include <time.h>

int main(int argc, char ** argv)
//
{
    //reals BenchmarkExampleDataX[6] {1.,2.,5.,7.,9.,3.};
    //reals BenchmarkExampleDataY[6] {7.,6.,8.,7.,5.,7.};
    //Data data1(6,BenchmarkExampleDataX,BenchmarkExampleDataY);

    reals BenchmarkExampleDataX[3] {1.1,3.2,2.1};
    reals BenchmarkExampleDataY[3] {2.2,2.3,3.2};
    Data data1(3,BenchmarkExampleDataX,BenchmarkExampleDataY);

    //reals BenchmarkExampleDataX[2] {1.,3.};
    //reals BenchmarkExampleDataY[2] {2.,2.};
    //Data data1(2,BenchmarkExampleDataX,BenchmarkExampleDataY);

    Circle circle;
    cout.precision(7);
    //
    circle = CircleFitByKasa (data1);
    cout << "\nTest One:\n  Kasa   fit:  center ("
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << endl;

    circle = CircleFitByPratt (data1);
    cout << "\n  Pratt  fit:  center ("
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << endl;

    circle = CircleFitByTaubin (data1);
    cout << "\n  Taubin fit:  center ("
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << endl;

    circle = CircleFitByHyper (data1);
    cout << "\n  Hyper  fit:  center ("
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << endl;
}

