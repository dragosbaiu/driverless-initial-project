#include "control.hpp"
#include "vehicle.hpp"
#include "environment.hpp"
#include "controller.hpp"
#include <cmath>

// Proportional control to compute steering angle based on heading error
double proportionalControl(Controller& controller, double targetHeadingAngle, double currentHeadingAngle){
    
    double error = targetHeadingAngle - currentHeadingAngle;
    double steeringAngle = controller.Kp * error;

    if (steeringAngle > controller.maxSteeringAngle) {
        steeringAngle = controller.maxSteeringAngle;
    } else if (steeringAngle < -controller.maxSteeringAngle) {
        steeringAngle = -controller.maxSteeringAngle;
    }
    return steeringAngle;
}

// Compute drift error based on vehicle's current position and heading
double computeDriftError(Vehicle& vehicle){
    return (-sin(vehicle.theta.front()) * (vehicle.x.back() - vehicle.x.front())) +
            ( cos(vehicle.theta.front()) * (vehicle.y.back() - vehicle.y.front()));
}

// Proportional control considering both heading error and lateral drift to compute steering angle
double proportionalControlWithLateralDrift(Vehicle& vehicle, Environment& environment, Controller& controller, double targetHeadingAngle, double currentHeadingAngle){
    
    double error = targetHeadingAngle - currentHeadingAngle;
    double errorDrift = computeDriftError(vehicle);
    double steeringAngle = controller.Kp * error - controller.Ky * errorDrift;

    if (steeringAngle > controller.maxSteeringAngle) {
        steeringAngle = controller.maxSteeringAngle;
    } else if (steeringAngle < -controller.maxSteeringAngle) {
        steeringAngle = -controller.maxSteeringAngle;
    }
    return steeringAngle;
}