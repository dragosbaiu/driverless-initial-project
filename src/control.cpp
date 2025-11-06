#include "control.hpp"
#include "vehicle.hpp"
#include "environment.hpp"
#include "controller.hpp"
#include <cmath>


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

double computeCrossTrackError(Vehicle& vehicle){
    return (-sin(vehicle.theta.front()) * (vehicle.x.back() - vehicle.x.front())) +
            ( cos(vehicle.theta.front()) * (vehicle.y.back() - vehicle.y.front()));
}

double proportionalControlWithLateralDrift(Vehicle& vehicle, Environment& environment, Controller& controller, double targetHeadingAngle, double currentHeadingAngle){
    
    double error = targetHeadingAngle - currentHeadingAngle;
    double errorDrift = computeCrossTrackError(vehicle);
    double steeringAngle = controller.Kp * error - controller.Ky * errorDrift;

    if (steeringAngle > controller.maxSteeringAngle) {
        steeringAngle = controller.maxSteeringAngle;
    } else if (steeringAngle < -controller.maxSteeringAngle) {
        steeringAngle = -controller.maxSteeringAngle;
    }
    return steeringAngle;
}