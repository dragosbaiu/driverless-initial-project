#include "sim.hpp"
#include "vehicle.hpp"
#include "path.hpp"
#include "control.hpp"
#include "environment.hpp"
#include "controller.hpp"
#include "noise.hpp"

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

// Simulate vehicle movement based on kinematic bicycle model, takes into account lateral drift
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
        vehicle.delta = computeHeadingCorrection(controller, targetHeadingAngle, vehicle.theta.back());
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
        // Add Gaussian noise to the current heading angle
        // The noise simulates the error in a heading sensor
        double noisyHeading = vehicle.theta.back() + generateGaussianNoise(controller);
        // Compute steering angle using proportional control, taking lateral drift and noise into consideration
        vehicle.delta = computeHeadingAndDriftCorrection(vehicle, environment, controller, targetHeadingAngle, noisyHeading);
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

// Generation of a straight path and returns the rounded maximum distance from origin
double simulateStraightPath (Path& path, Environment& environment){
    double max = 0.0;
    path.generateStraightPath();
    for (int i = 1; i < path.x.size(); i++){
        if (abs(path.x[i]) > max){
            max = abs(path.x[i]);
        }
        if (abs(path.y[i]) > max){
            max = abs(path.y[i]);
        }
    }

    return ceil(max);
}

// Generation of a circle path and returns the rounded maximum distance from origin
double simulateCirclePath (Path& path, Environment& environment){
    double max = 0.0;
    path.generateCirclePath();
    for (int i = 1; i < path.x.size(); i++){
        if (abs(path.x[i]) > max){
            max = abs(path.x[i]);
        }
        if (abs(path.y[i]) > max){
            max = abs(path.y[i]);
        }
    }

    return ceil(max);
}

// Generation of a sine path and returns the rounded maximum distance from origin
double simulateSinePath (Path& path, Environment& environment){
    double max = 0.0;
    path.generateSinePath();
    for (int i = 1; i < path.x.size(); i++){
        if (abs(path.x[i]) > max){
            max = abs(path.x[i]);
        }
        if (abs(path.y[i]) > max){
            max = abs(path.y[i]);
        }
    }

    return ceil(max);
}

// Simulate vehicle following a given path using path following control and returns the rounded maximum distance from origin
// (Task 3)
double simulatePathFollowingAndGetRoundedMax(Vehicle& vehicle, Path& path, Environment& environment, Controller& controller) {
    double max = 0.0;

    for (int i = 0; i < environment.steps; i++) {

        // --- Stop condition: vehicle close to last point ---
        double dx_end = vehicle.x.back() - path.x.back();
        double dy_end = vehicle.y.back() - path.y.back();
        double dist_to_end = std::sqrt(dx_end * dx_end + dy_end * dy_end);
        if (dist_to_end < vehicle.dt * vehicle.velocity.back() && i > 3) { 
            break;
        }
        vehicle.delta = computeSteeringForPathFollowing(vehicle, path, controller);
        vehicle.updatePosition();

        if (abs(vehicle.x.back()) > max) max = abs(vehicle.x.back());
        if (abs(vehicle.y.back()) > max) max = abs(vehicle.y.back());
    }

    return ceil(max);
}
