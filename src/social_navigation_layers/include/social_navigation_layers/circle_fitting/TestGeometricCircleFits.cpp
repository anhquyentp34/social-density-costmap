// It worked well
#include "mystuff.h"
#include "data.h"
#include "circle.h"
#include "Utilities.cpp"
#include "CircleFitByTaubin.cpp"
#include "CircleFitByPratt.cpp"
#include "CircleFitByKasa.cpp"
#include "CircleFitByHyper.cpp"
#include "CircleFitByChernovLesort.cpp"
#include "CircleFitByChernovHoussam.cpp"
#include "CircleFitByLevenbergMarquardtFull.cpp"
#include "CircleFitByLevenbergMarquardtReduced.cpp"
#include <time.h>

int main()
//             this code tests geometric circle fits
{
    int code;

    reals BenchmarkExampleDataX[6] {1.,2.,5.,7.,9.,3.};
    reals BenchmarkExampleDataY[6] {7.,6.,8.,7.,5.,7.};
    reals LambdaIni=0.01;

    Data data1(6,BenchmarkExampleDataX,BenchmarkExampleDataY);
    Circle circle,circleIni;
    cout.precision(7);
/*
       Test One:  benchmark example from the journal paper
    
       W. Gander, G. H. Golub, and R. Strebel,
       "Least squares fitting of circles and ellipses"
       BIT, volume 34, (1994), pages 558-578
          
          Correct answers:
          
       Taubin fit should give 
       center (4.613933,2.795209)  radius 4.879213  sigma 0.4572958
          
       All the four geometric fits should give
       center (4.739782,2.983533)  radius 4.714226  sigma 0.4523271
       and converge after 5-10 iterations
*/
    circleIni = CircleFitByTaubin (data1);
    cout << "\nTest One:\n  Initial circle by Taubin fit:\n  center (" 
         << circleIni.a <<","<< circleIni.b <<")  radius "
         << circleIni.r << "  sigma " << circleIni.s << endl;
    
    code = CircleFitByLevenbergMarquardtFull (data1,circleIni,LambdaIni,circle);
    if ((code == 1)||(code==2)) cout << "\n Geometric circle by Levenberg-Marquardt (full) did not converge. Iterations maxed out.\n";
    if (code == 3) cout << "\n Geometric circle by Levenberg-Marquardt (full) did not converge. Fitting circle too big.\n";
    if (code == 0) cout << "\n Geometric circle by Levenberg-Marquardt (full):\n  center (" 
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << "  iterations: " << circle.i << endl;
    
    code = CircleFitByLevenbergMarquardtReduced (data1,circleIni,LambdaIni,circle);
    if ((code == 1)||(code==2)) cout << "\n Geometric circle by Levenberg-Marquardt (reduced) did not converge. Iterations maxed out.\n";
    if (code == 3) cout << "\n Geometric circle by Levenberg-Marquardt (reduced) did not converge. Fitting circle too big.\n";
    if (code == 0) cout << "\n Geometric circle by Levenberg-Marquardt (reduced):\n  center (" 
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << "  iterations: " << circle.i << endl;
    
    code = CircleFitByChernovLesort (data1,circleIni,LambdaIni,circle);
    if ((code == 1)||(code==2)) cout << "\n Geometric circle by Chernov-Lesort did not converge. Iterations maxed out.\n";
    if (code == 3) cout << "\n Geometric circle by Chernov-Lesort did not converge. Fitting circle too big.\n";
    if (code == 0) cout << "\n Geometric circle by Chernov-Lesort:\n  center (" 
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << "  iterations: " << circle.i << endl;
    
    code = CircleFitByChernovHoussam (data1,circleIni,LambdaIni,circle);
    if ((code == 1)||(code==2)) cout << "\n Geometric circle by Chernov-Houssam did not converge. Iterations maxed out.\n";
    if (code == 3) cout << "\n Geometric circle by Chernov-Houssam did not converge. Fitting circle too big.\n";
    if (code == 0) cout << "\n Geometric circle by Chernov-Houssam:\n  center (" 
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << "  iterations: " << circle.i << endl;
    
//            Test Two:  a randomly generated data set
    
    Data data2(10);    //   specify the number of data points
    
//          use the c++ random number generator to simulate data coordinates
    
    srand ( (unsigned)time(NULL) );  //  seed the random generator
    SimulateRandom (data2,1.0);       //  this function is in Utilities.cpp

    circleIni = CircleFitByTaubin (data2);
    cout << "\nTest Two:\n  Initial circle by Taubin fit:\n  center (" 
         << circleIni.a <<","<< circleIni.b <<")  radius "
         << circleIni.r << "  sigma " << circleIni.s << endl;
    
    code = CircleFitByLevenbergMarquardtFull (data2,circleIni,LambdaIni,circle);
    if ((code == 1)||(code==2)) cout << "\n Geometric circle by Levenberg-Marquardt (full) did not converge. Iterations maxed out.\n";
    if (code == 3) cout << "\n Geometric circle by Levenberg-Marquardt (full) did not converge. Fitting circle too big.\n";
    if (code == 0) cout << "\n Geometric circle by Levenberg-Marquardt (full):\n  center (" 
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << "  iterations: " << circle.i << endl;
    
    code = CircleFitByLevenbergMarquardtReduced (data2,circleIni,LambdaIni,circle);
    if ((code == 1)||(code==2)) cout << "\n Geometric circle by Levenberg-Marquardt (reduced) did not converge. Iterations maxed out.\n";
    if (code == 3) cout << "\n Geometric circle by Levenberg-Marquardt (reduced) did not converge. Fitting circle too big.\n";
    if (code == 0) cout << "\n Geometric circle by Levenberg-Marquardt (reduced):\n  center (" 
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << "  iterations: " << circle.i << endl;
    
    code = CircleFitByChernovLesort (data2,circleIni,LambdaIni,circle);
    if ((code == 1)||(code==2)) cout << "\n Geometric circle by Chernov-Lesort did not converge. Iterations maxed out.\n";
    if (code == 3) cout << "\n Geometric circle by Chernov-Lesort did not converge. Fitting circle too big.\n";
    if (code == 0) cout << "\n Geometric circle by Chernov-Lesort:\n  center (" 
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << "  iterations: " << circle.i << endl;
    
    code = CircleFitByChernovHoussam (data2,circleIni,LambdaIni,circle);
    if ((code == 1)||(code==2)) cout << "\n Geometric circle by Chernov-Houssam did not converge. Iterations maxed out.\n";
    if (code == 3) cout << "\n Geometric circle by Chernov-Houssam did not converge. Fitting circle too big.\n";
    if (code == 0) cout << "\n Geometric circle by Chernov-Houssam:\n  center (" 
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << "  iterations: " << circle.i << endl;
/*
       Test Three:  a special sample of five points
       
       This is a modified "W-example" from Section 5.13 of the book by Nikolai Chernov:
       "Circular and linear regression: Fitting circles and lines by least squares"
       Chapman & Hall/CRC, Monographs on Statistics and Applied Probability, volume 117, 2010.
       
       In this example, the algebraic fit by Taubin mistakenly places 
       the initial circle on the "wrong side" of the sample 
       (in fact, most algebraic fits do the same in this example).
       
       As a result, both versions of the standard Levenberg-Marquardt algorithm
       move the fitting circle in the wrong direction and diverge.
       
       Chernov-Lesort and Chernov-Houssam methods, designed to converge 
       from any initial circle, flip the fitting circle to the "right side" 
       of the sample and find the best fitting circle
          
          Correct answers:
          
       Taubin fit should give 
       center (0.0,1237.1)  radius 1237.  sigma  1.09612
          
       The best fitting circle is 
       center (0.0,-3283.3)  radius 3283.3  
       
       sigma computed by a standard, numerically unstable formula is 1.09579
       sigma computed by a special,  numerically stable   formula is 1.20075
*/
    
    reals epsilon = 0.001;
    Data data3(5);
    data3.X[0] = 0.;  data3.Y[0] = 2.;
    data3.X[1] = 1.; data3.X[2] = -data3.X[1]; 
    data3.Y[1] = data3.Y[2] = -(1.+epsilon);
    data3.X[3] = 10.;  data3.X[4] = -data3.X[3];
    data3.Y[3] = data3.Y[4] = epsilon;

    circleIni = CircleFitByTaubin (data3);
    cout << "\nTest Three:\n  Initial circle by Taubin fit:\n  center (" 
         << circleIni.a <<","<< circleIni.b <<")  radius "
         << circleIni.r << "  sigma " << circleIni.s << endl;
    
    code = CircleFitByLevenbergMarquardtFull (data3,circleIni,LambdaIni,circle);
    if ((code == 1)||(code==2)) cout << "\n Geometric circle by Levenberg-Marquardt (full) did not converge. Iterations maxed out.\n";
    if (code == 3) cout << "\n Geometric circle by Levenberg-Marquardt (full) did not converge. Fitting circle too big.\n";
    if (code == 0) cout << "\n Geometric circle by Levenberg-Marquardt (full):\n  center (" 
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << "  iterations: " << circle.i << endl;
    
    code = CircleFitByLevenbergMarquardtReduced (data3,circleIni,LambdaIni,circle);
    if ((code == 1)||(code==2)) cout << "\n Geometric circle by Levenberg-Marquardt (reduced) did not converge. Iterations maxed out.\n";
    if (code == 3) cout << "\n Geometric circle by Levenberg-Marquardt (reduced) did not converge. Fitting circle too big.\n";
    if (code == 0) cout << "\n Geometric circle by Levenberg-Marquardt (reduced):\n  center (" 
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << "  iterations: " << circle.i << endl;
    
    code = CircleFitByChernovLesort (data3,circleIni,LambdaIni,circle);
    if ((code == 1)||(code==2)) cout << "\n Geometric circle by Chernov-Lesort did not converge. Iterations maxed out.\n";
    if (code == 3) cout << "\n Geometric circle by Chernov-Lesort did not converge. Fitting circle too big.\n";
    if (code == 0) cout << "\n Geometric circle by Chernov-Lesort:\n  center (" 
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << "  iterations: " << circle.i << endl;
    
    code = CircleFitByChernovHoussam (data3,circleIni,LambdaIni,circle);
    if ((code == 1)||(code==2)) cout << "\n Geometric circle by Chernov-Houssam did not converge. Iterations maxed out.\n";
    if (code == 3) cout << "\n Geometric circle by Chernov-Houssam did not converge. Fitting circle too big.\n";
    if (code == 0) cout << "\n Geometric circle by Chernov-Houssam:\n  center (" 
         << circle.a <<","<< circle.b <<")  radius "
         << circle.r << "  sigma " << circle.s << "  iterations: " << circle.i << endl;
}
