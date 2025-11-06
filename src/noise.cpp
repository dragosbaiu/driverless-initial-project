#include "noise.hpp"
#include <cmath>
#include <cstdlib>

// Generate Gaussian noise using Box-Muller transform (method that is efficient for generating normally distributed random numbers)
double generateGaussianNoise(Controller controller){
    
    static bool hasSpare = false;
    static double spare;

    if(hasSpare){
        hasSpare = false;
        return controller.mean + controller.standardDeviation * spare;
    }

    hasSpare = true;
    static double u, v, s;
    do{
        u = (rand() / ((double) RAND_MAX)) * 2.0 - 1.0;
        v = (rand() / ((double) RAND_MAX)) * 2.0 - 1.0;
        s = u * u + v * v;
    } while( s >= 1.0 || s == 0.0 );

    s = sqrt( -2.0 * log(s) / s );
    spare = v * s;
    return controller.mean + controller.standardDeviation * (u * s);
}