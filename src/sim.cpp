#include "sim.hpp"
#include "vehicle.hpp"
#include "path.hpp"
#include "control.hpp"
#include "environment.hpp"
#include "controller.hpp"
#include "noise.hpp"

using namespace std;

// Each function simulates the vehicle/path, updating its state after each time step
// The maximum distance from the origin is returned

// Simulates vehicle movement based on kinematic bicycle model
// Returns the rounded maximum distance from origin
// (Task 1)
double simulateBase(Vehicle& vehicle, Environment& environment){
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

// Simulates vehicle movement based on kinematic bicycle model, takes into account lateral drift
// Returns the rounded maximum distance from origin
// (Task 1)
double simulateBaseDrift(Vehicle& vehicle, Environment& environment){

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

// Simulates vehicle moving in a straight path using proportional control
// Returns the rounded maximum distance from origin
// (Task 2)
double simulateHeadingStraight(Vehicle& vehicle, Environment& environment, Controller& controller){

    double max = 0.0;
    double targetHeadingAngle = vehicle.theta[0];

    for (int i = 0; i < environment.steps; i++){
        // Compute steering angle using proportional control
        vehicle.delta = steerHeading(controller, targetHeadingAngle, vehicle.theta.back());
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

// Simulates vehicle moving in a straight path using proportional control, takes into account lateral drift and noise.
// Returns the rounded maximum distance from origin
// (Task 2)
double simulateHeadingStraightDrift(Vehicle& vehicle, Environment& environment, Controller& controller){

    double max = 0.0;

    double targetHeadingAngle = vehicle.theta[0];

    for (int i = 0; i < environment.steps; i++){
        // Add Gaussian noise to the current heading angle
        double noisyHeading = vehicle.theta.back() + generateGaussianNoise(controller);
        // Compute steering angle using proportional control, taking lateral drift and noise into consideration
        vehicle.delta = steerHeadingDrift(vehicle, environment, controller, targetHeadingAngle, noisyHeading);
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

// Generates a straight path
// Returns the rounded maximum distance from origin
// (Task 3)
double simulateStraightPath(Path& path, Environment& environment){
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

// Generates a circular path
// Returns the rounded maximum distance from origin
// (Task 3)
double simulateCircularPath(Path& path, Environment& environment){
    double max = 0.0;
    path.generateCircularPath();
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

// Generates a sine path
// Returns the rounded maximum distance from origin
// (Task 3)
double simulateSinePath(Path& path, Environment& environment){
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

// Simulates vehicle following a given path 
// Uses P controller 
// Returns the rounded maximum distance from origin
// (Task 3)
double simulateP(Vehicle& vehicle, Path& path, Environment& environment, Controller& controller) {

    double max = 0.0;

    for (int i = 0; i < environment.stepsPath; i++) {

        double vx = vehicle.x.back();
        double vy = vehicle.y.back();

        int closestIndex = 0;
        double minDist2 = 1e9;

        for (int k = 1; k < (int)path.x.size(); k++) {
            double dx = vx - path.x[k];
            double dy = vy - path.y[k];
            double dist2 = dx*dx + dy*dy;
            if (dist2 < minDist2) {
                minDist2 = dist2;
                closestIndex = k;
            }
        }

        if (closestIndex >= (int)path.x.size() - 2 && i > 5) {
            break;
        }

        vehicle.delta = steerHeadingP(vehicle, path, controller);
        vehicle.updatePosition();
        vehicle.applyLateralDrift(environment);

        if (abs(vehicle.x.back()) > max) max = abs(vehicle.x.back());
        if (abs(vehicle.y.back()) > max) max = abs(vehicle.y.back());
    }

    return ceil(max);
}

// Simulates vehicle following a given path
// Uses PID controller 
// Returns the rounded maximum distance from origin
// (Task 4)
double simulatePID(Vehicle& vehicle, Path& path, Environment& environment, Controller& controller) {

    double max = 0.0;

    for (int i = 0; i < environment.stepsPath; i++) {

        double vx = vehicle.x.back();
        double vy = vehicle.y.back();

        int closestIndex = 0;
        double minDist2 = 1e9;

        for (int k = 0; k < (int)path.x.size(); ++k) {
            double dx = vx - path.x[k];
            double dy = vy - path.y[k];
            double dist2 = dx*dx + dy*dy;
            if (dist2 < minDist2) {
                minDist2 = dist2;
                closestIndex = k;
            }
        }

        if (closestIndex >= (int)path.x.size() - 2 && i > 5) {
            break;
        }

        vehicle.delta = steerHeadingPID(vehicle, path, controller);
        vehicle.updatePosition();
        vehicle.applyLateralDrift(environment);

        if (abs(vehicle.x.back()) > max) max = abs(vehicle.x.back());
        if (abs(vehicle.y.back()) > max) max = abs(vehicle.y.back());
    }

    return ceil(max);
}

// Generates a chicane path 
// Returns the rounded maximum distance from origin
double simulateChicanePath(Path& path, Environment& environment)
{
    double max = 0.0;

    path.generateChicanePath();

    for (size_t i = 0; i < path.x.size(); ++i) {
        if (abs(path.x[i]) > max) max = abs(path.x[i]);
        if (abs(path.y[i]) > max) max = abs(path.y[i]);
    }

    return ceil(max);
}


// Simulates vehicle following a given path 
// Uses Stanley ontroller 
// Returns the rounded maximum distance from origin
// (Task 5)
double simulateStanley(Vehicle& vehicle, Path& path, Environment& environment, Controller& controller) {

    double max = 0.0;

    for (int i = 0; i < environment.stepsPath; i++) {

        double vx = vehicle.x.back();
        double vy = vehicle.y.back();

        int closestIndex = 0;
        double minDist2 = 1e9;

        for (int k = 0; k < (int)path.x.size(); ++k) {
            double dx = vx - path.x[k];
            double dy = vy - path.y[k];
            double dist2 = dx*dx + dy*dy;
            if (dist2 < minDist2) {
                minDist2 = dist2;
                closestIndex = k;
            }
        }

        if (closestIndex >= (int)path.x.size() - 2 && i > 5) {
            break;
        }

        vehicle.delta = steerHeadingStanley(vehicle, path, controller);
        vehicle.updatePosition();
        vehicle.applyLateralDrift(environment);

        if (abs(vehicle.x.back()) > max) max = abs(vehicle.x.back());
        if (abs(vehicle.y.back()) > max) max = abs(vehicle.y.back());
    }

    return ceil(max);
}