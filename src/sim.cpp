#include "sim.hpp"
#include "vehicle.hpp"
#include "control.hpp"
#include "environment.hpp"
#include "controller.hpp"
#include <cmath>

using namespace std;

double simulateAndGetRoundedMax(Vehicle& vehicle, Environment& environment){
    double max = 0.0;

    for (int i = 1; i < environment.steps; i++){
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

double simulateWithDriftAndGetRoundedMax(Vehicle& vehicle, Environment& environment){

    double max = 0.0;

    for (int i = 1; i < environment.steps; i++){
        vehicle.updatePosition();
        vehicle.applyLateralDrift(environment);
        if (abs(vehicle.x[i]) > max){
            max = abs(vehicle.x[i]);
        }
        if (abs(vehicle.y[i]) > max){
            max = abs(vehicle.y[i]);
        }
    }

    return ceil(max);
}


double simulateStraightPathAndGetRoundedMax(Vehicle& vehicle, Environment& environment, Controller& controller){

    double max = 0.0;

    double targetHeadingAngle = vehicle.theta[0];

    for (int i = 0; i < environment.steps; i++){
        vehicle.delta = proportionalControl(controller, targetHeadingAngle, vehicle.theta.back());
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

double simulateStraightPathWithDriftAndGetRoundedMax(Vehicle& vehicle, Environment& environment, Controller& controller){

    double max = 0.0;

    double targetHeadingAngle = vehicle.theta[0];

    for (int i = 0; i < environment.steps; i++){
        vehicle.delta = proportionalControlWithLateralDrift(vehicle, environment, controller, targetHeadingAngle, vehicle.theta.back());
        vehicle.updatePosition();
        vehicle.applyLateralDrift(environment);
        if (abs(vehicle.x[i]) > max){
            max = abs(vehicle.x[i]);
        }
        if (abs(vehicle.y[i]) > max){
            max = abs(vehicle.y[i]);
        }
    }

    return ceil(max);
}

