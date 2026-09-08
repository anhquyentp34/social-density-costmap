int CircleFitByLevenbergMarquardtReduced (Data& data, Circle& circleIni, reals LambdaIni, Circle& circle)
/*                                        <------------------ Input ------------------->  <-- Output -->

       Geometric circle fit to a given set of data points (in 2D)
		
       Input:  data     - the class of data (contains the given points):
		
	       data.n   - the number of data points
	       data.X[] - the array of X-coordinates
	       data.Y[] - the array of Y-coordinates
		          
               circleIni - parameters of the initial circle ("initial guess")
		        
	       circleIni.a - the X-coordinate of the center of the initial circle
	       circleIni.b - the Y-coordinate of the center of the initial circle
	       circleIni.r - the radius of the initial circle
		        
	       LambdaIni - the initial value of the control parameter "lambda"
	                   for the Levenberg-Marquardt procedure
	                   (common choice is a small positive number, e.g. 0.001)
		        
       Output:
	       integer function value is a code:
	                  0:  normal termination, the best fitting circle is 
	                      successfully found
	                  1:  the number of outer iterations exceeds the limit (99)
	                      (indicator of a possible divergence)
	                  2:  the number of inner iterations exceeds the limit (99)
	                      (another indicator of a possible divergence)
	                  3:  the coordinates of the center are too large
	                      (a strong indicator of divergence)
		                   
	       circle - parameters of the fitting circle ("best fit")
		        
	       circle.a - the X-coordinate of the center of the fitting circle
	       circle.b - the Y-coordinate of the center of the fitting circle
 	       circle.r - the radius of the fitting circle
 	       circle.s - the root mean square error (the estimate of sigma)
 	       circle.i - the total number of outer iterations (updating the parameters)
 	       circle.j - the total number of inner iterations (adjusting lambda)
 		        
       Algorithm:  Levenberg-Marquardt running over the "reduced" parameter space (a,b)
                   (the radius r is always set to its optimal value)
                         
       See a detailed description in Section 4.6 of the book by Nikolai Chernov:
       "Circular and linear regression: Fitting circles and lines by least squares"
       Chapman & Hall/CRC, Monographs on Statistics and Applied Probability, volume 117, 2010.
         
		Nikolai Chernov,  February 2014
*/
{
    int code,i,pivot,iter,inner,IterMAX=99;

    reals factorUp=10.,factorDown=0.04,lambda,ParLimit=1.e+6;
    reals dx,dy,ri,u,v;
    reals Mu,Mv,Muu,Mvv,Muv,Mr,A11,A12,A22,F1,F2,dX,dY;
    reals epsilon=3.e-8;
    reals G11,G12,G22,D1,D2;

    Circle Old,New;

    data.means();   // Compute x- and y-means (via a function in class "data") 
    
//       starting with the given initial circle (initial guess)

    New = circleIni;
    
//     compute the root-mean-square error via function SigmaReduced; see Utilities.cpp

    New.s = SigmaReduced(data,New);

//     initializing lambda, iteration counters, and the exit code

    lambda = LambdaIni;
    iter = inner = code = 0;

NextIteration:

    Old = New;
    if (++iter > IterMAX) {code = 1;  goto enough;}

//     computing moments

    Mu=Mv=Muu=Mvv=Muv=Mr=0.;

    for (i=0; i<data.n; i++)
    {
        dx = data.X[i] - Old.a;
        dy = data.Y[i] - Old.b;
        ri = sqrt(dx*dx + dy*dy);
        u = dx/ri;
        v = dy/ri;
        Mu += u;
        Mv += v;
        Muu += u*u;
        Mvv += v*v;
        Muv += u*v;
        Mr += ri;
    }
    Mu /= data.n;
    Mv /= data.n;
    Muu /= data.n;
    Mvv /= data.n;
    Muv /= data.n;
    Mr /= data.n;

//    computing matrices

    F1 = Old.a + Mu*Mr - data.meanX;
    F2 = Old.b + Mv*Mr - data.meanY;

try_again:

    A11 = (Muu - Mu*Mu) + lambda;
    A22 = (Mvv - Mv*Mv) + lambda;
    A12 = Muv - Mu*Mv;

    if (A11<epsilon || A22<epsilon || A11*A22-A12*A12<epsilon)
    {
        lambda *= factorUp;
        goto try_again;
    }

//      Cholesky decomposition with pivoting

    pivot=0;
    if (A11 < A22)
    {
       	swap(A11,A22);
    	swap(F1,F2);
        pivot=1;
    }

    G11 = sqrt(A11);
    G12 = A12/G11;
    G22 = sqrt(A22 - G12*G12);

    D1 = F1/G11;
    D2 = (F2 - G12*D1)/G22;

//    updating the parameters

    dY = D2/G22;
    dX = (D1 - G12*dY)/G11;
    
    if ((abs(dX)+abs(dY))/(One+abs(Old.a)+abs(Old.b)) < epsilon) goto enough;

    if (pivot != 0) swap(dX,dY);

    New.a = Old.a - dX;
    New.b = Old.b - dY;

    if (abs(New.a)>ParLimit || abs(New.b)>ParLimit) {code = 3; goto enough;}
    
//    compute the root-mean-square error via function SigmaReduced; see Utilities.cpp

    New.s = SigmaReduced(data,New);

//      check if improvement is gained

    if (New.s < Old.s)    //    yes, improvement
    {
        lambda *= factorDown;
        goto NextIteration;
    }
    else                  //  no improvement
    {
        if (++inner > IterMAX) {code = 2;  goto enough;}
        lambda *= factorUp;
        goto try_again;
    }

//       exit

enough:
    
//    compute the optimal radius via a function in Utilities.cpp

    Old.r = OptimalRadius(data,Old);
    Old.i = iter;
    Old.j = inner;
    
    circle = Old;
    
    return code;
}

