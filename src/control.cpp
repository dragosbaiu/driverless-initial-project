#include "control.hpp"
#include "vehicle.hpp"
#include "environment.hpp"
#include "controller.hpp"
#include "path.hpp"
#include <cmath>

using namespace std;

// Proportional control to compute steering angle based on heading error and keep the vehicle on a straight path
double computeHeadingCorrection(Controller& controller, double targetHeadingAngle, double currentHeadingAngle){
    
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

// Proportional control considering both heading error and lateral drift to compute steering angle and keep the vehicle on a straight path
double computeHeadingAndDriftCorrection(Vehicle& vehicle, Environment& environment, Controller& controller, double targetHeadingAngle, double currentHeadingAngle){
    
    double error = targetHeadingAngle - currentHeadingAngle;
    double errorDrift = computeDriftError(vehicle);
    double steeringAngle = controller.Kp * error - controller.Kpx * errorDrift;

    if (steeringAngle > controller.maxSteeringAngle) {
        steeringAngle = controller.maxSteeringAngle;
    } else if (steeringAngle < -controller.maxSteeringAngle) {
        steeringAngle = -controller.maxSteeringAngle;
    }
    return steeringAngle;
}

// Compute cross-track error between vehicle position and the predefined path
double computeCrossTrackError(Vehicle& vehicle, Path& path){
    // Closest point on path to vehicle
    double minDistance = 1e9;
    int closestIndex = 0;

    double vx = vehicle.x.back();
    double vy = vehicle.y.back();

    for (int i = 0; i < (int)path.x.size(); i++){
        double dx = vx - path.x[i];
        double dy = vy - path.y[i];
        double distance = sqrt(dx*dx + dy*dy);
        if (distance < minDistance){
            minDistance = distance;
            closestIndex = i;
        }
    }

    // Use closest point and its neighbor to compute path heading
    int i = closestIndex;
    int j;
    if (i == 0) {
        j = 1;                 
    } else if (i == (int)path.x.size() - 1) {
        j = i - 1;     
    } else {
        j = i + 1;                    
    }

    double dxPath = path.x[j] - path.x[i];
    double dyPath = path.y[j] - path.y[i];
    double pathHeading = std::atan2(dyPath, dxPath);

    // Compute cross-track error using vehicle position and path heading
    double dx = vx - path.x[i];
    double dy = vy - path.y[i];

    double crossTrackError = sin(pathHeading) * dx - cos(pathHeading) * dy;

    return crossTrackError;
}

// Compute steering angle for path following using P control
double computeSteeringForPathFollowingP(Vehicle& vehicle, Path& path, Controller& controller){
    double crossTrackError = computeCrossTrackError(vehicle, path);
    double steeringAngle = controller.Kpx* crossTrackError;

    if (steeringAngle > controller.maxSteeringAngle) {
        steeringAngle = controller.maxSteeringAngle;
    } else if (steeringAngle < -controller.maxSteeringAngle) {
        steeringAngle = -controller.maxSteeringAngle;
    }
    return steeringAngle;
}



// Reset the PID controller state for path following
void resetPathPIDState(Controller& controller) {
    controller.integralError = 0.0;
    controller.previousError = 0.0;
    controller.firstUpdate = true;
}

// Compute steering angle for path following using PID control
double computeSteeringForPathFollowingPID(Vehicle& vehicle, Path& path, Controller& controller) {
    double crossTrackError = computeCrossTrackError(vehicle, path);

    controller.integralError += crossTrackError * vehicle.dt;

    double derivative = 0.0;
    if (!controller.firstUpdate) {
        derivative = (crossTrackError - controller.previousError) / vehicle.dt;
    } else {
        controller.firstUpdate = false;
    }

    controller.previousError = crossTrackError;

    double steeringAngle = controller.Kpx * crossTrackError + controller.Ki  * controller.integralError + controller.Kd  * derivative;

    if (steeringAngle > controller.maxSteeringAngle) {
        steeringAngle = controller.maxSteeringAngle;
    } else if (steeringAngle < -controller.maxSteeringAngle) {
        steeringAngle = -controller.maxSteeringAngle;
    }

    return steeringAngle;
}

// Compute cross-track error and path heading angle (Same as the other function but returns the path heading as well)
PathError computeCrossTrackErrorAndPathHeading(Vehicle& vehicle, Path& path){

    PathError result;

    // Closest point on path to vehicle
    double minDistance = 1e9;
    int closestIndex = 0;

    double vx = vehicle.x.back();
    double vy = vehicle.y.back();

    for (int i = 0; i < (int)path.x.size(); i++){
        double dx = vx - path.x[i];
        double dy = vy - path.y[i];
        double distance = sqrt(dx*dx + dy*dy);
        if (distance < minDistance){
            minDistance = distance;
            closestIndex = i;
        }
    }

    // Use closest point and its neighbor to compute path heading
    int i = closestIndex;
    int j;
    if (i == 0) {
        j = 1;                 
    } else if (i == (int)path.x.size() - 1) {
        j = i - 1;     
    } else {
        j = i + 1;                    
    }

    double dxPath = path.x[j] - path.x[i];
    double dyPath = path.y[j] - path.y[i];
    double pathHeading = std::atan2(dyPath, dxPath);
    result.pathHeading = pathHeading;

    // Compute cross-track error using vehicle position and path heading
    double dx = vx - path.x[i];
    double dy = vy - path.y[i];

    double crossTrackError = sin(pathHeading) * dx - cos(pathHeading) * dy;
    result.crossTrackError = crossTrackError;

    return result;
}

// Keep angle in between -pi and pi
double normalizeAngle(double angle) {
    while (angle > M_PI)  angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// Compute steering angle for path following using Stanely control
double computeSteeringForStanley(Vehicle& vehicle, Path& path, Controller& controller) {

    PathError err = computeCrossTrackErrorAndPathHeading(vehicle, path);
    double e = err.crossTrackError;
    double pathHeading = err.pathHeading;

    double vehicleHeading = vehicle.theta.back();

    double headingError = normalizeAngle(pathHeading - vehicleHeading);

    double v = vehicle.velocity.back(); 

    v = std::fabs(v);
    if (v < 0.1) {
        v = 0.1;  
    }

    double stanleyTerm = std::atan2(controller.Ks * e, v);
    double steeringAngle = headingError + stanleyTerm;

    if (steeringAngle > controller.maxSteeringAngle) {
        steeringAngle = controller.maxSteeringAngle;
    } else if (steeringAngle < -controller.maxSteeringAngle) {
        steeringAngle = -controller.maxSteeringAngle;
    }

    return steeringAngle;
}