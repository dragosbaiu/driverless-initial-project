#include "sim.hpp"
#include "vehicle.hpp"
#include "control.hpp"
#include <cmath>

using namespace std;

double simulateAndGetRoundedMax(Vehicle& vehicle, int steps){

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

double simulateStraightPathAndGetRoundedMax(Vehicle& vehicle, int steps){

    double max = 0.0;

    double Kp = 0.5;
    double targetHeadingAngle = vehicle.theta[0];
    double maxSteeringAngle = 0.5;

    for (int i = 0; i < steps; i++){
        vehicle.delta = proportionalControl(targetHeadingAngle, vehicle.theta.back(), Kp, maxSteeringAngle);
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