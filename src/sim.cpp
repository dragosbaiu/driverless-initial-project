#include "sim.hpp"
#include "vehicle.hpp"
#include <cmath>

using namespace std;

double simulate_and_get_rounded_max(Vehicle& vehicle, int steps){

    double max = 0.0;

    for (int i = 1; i < steps; i++){
        vehicle.updatePosition();
        if (abs(vehicle.x[i]) > max){
            max = abs(vehicle.x[i]);
        }
        if (abs(vehicle.y[i]) > max){
            max = abs(vehicle.y[i]);
        }
    }

    return ceil(max);
}