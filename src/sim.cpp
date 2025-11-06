#include "sim.hpp"
#include "vehicle.hpp"
#include "control.hpp"
#include "environment.hpp"
#include "controller.hpp"
#include <cmath>

using namespace std;

// Simulate vehicle movement based on kinematic bicycle model and returns the rounded maximum distance from origin
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

// Simulate vehicle movement based on kinematic bicycle model, takes into account lateral drift, and noise
// Returns the rounded maximum distance from origin
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

// Simulate vehicle moving in a straight path using proportional control and returns the rounded maximum distance from origin
double simulateStraightPathAndGetRoundedMax(Vehicle& vehicle, Environment& environment, Controller& controller){

    double max = 0.0;

    double targetHeadingAngle = vehicle.theta[0];

    for (int i = 0; i < environment.steps; i++){
        // Compute steering angle using proportional control
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

// Simulate vehicle moving in a straight path using proportional control, takes into account lateral drift and noise.
// Returns the rounded maximum distance from origin
double simulateStraightPathWithDriftAndGetRoundedMax(Vehicle& vehicle, Environment& environment, Controller& controller){

    double max = 0.0;

    double targetHeadingAngle = vehicle.theta[0];

    for (int i = 0; i < environment.steps; i++){
        // Compute steering angle using proportional control, taking lateral drift and noise into consideration
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

