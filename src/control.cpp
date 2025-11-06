#include "control.hpp"
#include "vehicle.hpp"
#include <cmath>


double proportionalControl(double targetHeadingAngle, double currentHeadingAngle, double Kp, double maxSteeringAngle){
    
    double error = targetHeadingAngle - currentHeadingAngle;
    double steeringAngle = Kp * error;

    if (steeringAngle > maxSteeringAngle) {
        steeringAngle = maxSteeringAngle;
    } else if (steeringAngle < -maxSteeringAngle) {
        steeringAngle = -maxSteeringAngle;
    }
    return steeringAngle;
}

double computeCrossTrackError(const Vehicle& vehicle){
    return (-sin(vehicle.theta.front()) * (vehicle.x.back() - vehicle.x.front())) +
            ( cos(vehicle.theta.front()) * (vehicle.y.back() - vehicle.y.front()));
}

double proportionalControlWithLateralDrift(const Vehicle& vehicle, double targetHeadingAngle, double currentHeadingAngle, double Kp, double maxSteeringAngle, double lateralDrift, double Ky){
    
    double error = targetHeadingAngle - currentHeadingAngle;
    double errorDrift = computeCrossTrackError(vehicle);
    double steeringAngle = Kp * error - Ky * errorDrift;

    if (steeringAngle > maxSteeringAngle) {
        steeringAngle = maxSteeringAngle;
    } else if (steeringAngle < -maxSteeringAngle) {
        steeringAngle = -maxSteeringAngle;
    }
    return steeringAngle;
}