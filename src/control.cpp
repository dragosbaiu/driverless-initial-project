#include "control.hpp"


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