#include "vehicle.hpp"
#include "io.hpp"
#include "sim.hpp"
#include "graphics.hpp"
#include "control.hpp"
#include "noise.hpp"
#include "controller.hpp"
#include "environment.hpp"
#include "path.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

using namespace std;

int main() {
    
    Vehicle vehicle;
    Controller controller;
    Environment environment;

    if (!readInput("data/input.txt", vehicle, controller, environment)) {
        return 1;
    }

    // Create copies of the vehicle for different simulation scenarios
    Vehicle standardVehicle = vehicle;
    Vehicle standardVehicleWithDrift = vehicle;
    Vehicle controlledVehicle = vehicle;
    Vehicle controlledVehicleWithDrift = vehicle;
    Vehicle controlledVehicleWithPredefinedStraightPath = vehicle;
    Vehicle controlledVehicleWithPredefinedCircularPath = vehicle;
    Vehicle controlledVehicleWithPredefinedSinePath = vehicle;
    Vehicle controlledPIDVehicleWithPredefinedStraightPath = vehicle;
    Vehicle controlledPIDVehicleWithPredefinedCircularPath = vehicle;
    Vehicle controlledPIDVehicleWithPredefinedSinePath = vehicle;
    Vehicle controlledPIDVehicleWithPredefinedSharpTurnPath = vehicle;
    Vehicle controlledStanleyVehicleWithPredefinedStraightPath = vehicle;
    Vehicle controlledStanleyVehicleWithPredefinedCircularPath = vehicle;
    Vehicle controlledStanleyVehicleWithPredefinedSinePath = vehicle;
    Vehicle controlledStanleyVehicleWithPredefinedSharpTurnPath = vehicle;

    // Simulate and visualize standard bicycle model
    double roundedStandardMax = simulateAndGetRoundedMax(standardVehicle, environment);
    runApp(standardVehicle, roundedStandardMax, "Trajectory of Standard Bicycle Model");

    // Simulate and visualize bicycle model with lateral drift
    double roundedDriftMax = simulateWithDriftAndGetRoundedMax(standardVehicleWithDrift, environment);
    runApp(standardVehicleWithDrift, roundedDriftMax, "Trajectory of Bicycle with Lateral Drift");

    // Simulate and visualize controlled bicycle model
    double roundedControlledMax = simulateStraightPathAndGetRoundedMax(controlledVehicle, environment, controller);
    runApp(controlledVehicle, roundedControlledMax, "Trajectory of Controlled Bicycle");

    // Simulate and visualize controlled bicycle model with noise and lateral drift
    double roundedControlledDriftMax = simulateStraightPathWithDriftAndGetRoundedMax(controlledVehicleWithDrift, environment, controller);
    runApp(controlledVehicleWithDrift, roundedControlledDriftMax, "Trajectory of Controlled Bicycle with Noise and Lateral Drift");


    double initialX = 0;
    double initialY = 0;
    // Generate and visualize a straight path, with fields: x, y, theta, lenght, dt
    // Uses P controller to follow the path
    Path path(initialX, initialY, 1, 100, vehicle.dt*1.5);

    double roundedStraightPathMax = simulateStraightPath(path, environment);
    if (path.x.size() >= 2) {
        controlledVehicleWithPredefinedStraightPath.setInitialTheta(atan2(path.y[1] - path.y[0], path.x[1] - path.x[0]));
    } else {
        controlledVehicleWithPredefinedStraightPath.setInitialTheta(0);
    }
    controlledVehicleWithPredefinedStraightPath.setInitialX(path.x.front());
    controlledVehicleWithPredefinedStraightPath.setInitialY(path.y.front());

    double vehicleRoundedStraightPathMax = simulatePathFollowingPAndGetRoundedMax(controlledVehicleWithPredefinedStraightPath, path, environment, controller);
    runApp(controlledVehicleWithPredefinedStraightPath, path, max (roundedStraightPathMax, vehicleRoundedStraightPathMax), "Generated Straight Path And Controlled Vehicle Trajectory");

    // Generate and visualize a straight path, with fields: x, y, theta, lenght, dt
    // Uses PID controller to follow the path
    Path pathPID(initialX, initialY, 1, 100, vehicle.dt*1.5);
    resetPathPIDState(controller);
    double roundedStraightPathMaxPID = simulateStraightPath(pathPID, environment);
    if (pathPID.x.size() >= 2) {
        controlledPIDVehicleWithPredefinedStraightPath.setInitialTheta(atan2(pathPID.y[1] - pathPID.y[0], pathPID.x[1] - pathPID.x[0]));
    } else {
        controlledPIDVehicleWithPredefinedStraightPath.setInitialTheta(0);
    }
    controlledPIDVehicleWithPredefinedStraightPath.setInitialX(pathPID.x.front());
    controlledPIDVehicleWithPredefinedStraightPath.setInitialY(pathPID.y.front());

    double vehicleRoundedStraightPathMaxPID = simulatePathFollowingPIDAndGetRoundedMax(controlledPIDVehicleWithPredefinedStraightPath, pathPID, environment, controller);
    runApp(controlledPIDVehicleWithPredefinedStraightPath, pathPID, max (roundedStraightPathMaxPID, vehicleRoundedStraightPathMaxPID), "Generated Straight Path And PID Controlled Vehicle Trajectory");

    // Generate and visualize a straight path, with fields: x, y, theta, lenght, dt
    // Uses Stanely controller to follow the path
    Path pathStanley(initialX, initialY, 1, 100, vehicle.dt*1.5);
    double roundedStraightPathMaxStanley = simulateStraightPath(pathStanley, environment);
    if (pathStanley.x.size() >= 2) {
        controlledStanleyVehicleWithPredefinedStraightPath.setInitialTheta(atan2(pathStanley.y[1] - pathStanley.y[0], pathStanley.x[1] - pathStanley.x[0]));
    } else {
        controlledStanleyVehicleWithPredefinedStraightPath.setInitialTheta(0);
    }
    controlledStanleyVehicleWithPredefinedStraightPath.setInitialX(pathStanley.x.front());
    controlledStanleyVehicleWithPredefinedStraightPath.setInitialY(pathStanley.y.front());

    double vehicleRoundedStraightPathMaxStanley = simulatePathFollowingStanleyAndGetRoundedMax(controlledStanleyVehicleWithPredefinedStraightPath, pathStanley, environment, controller);
    runApp(controlledStanleyVehicleWithPredefinedStraightPath, pathStanley, max (roundedStraightPathMaxStanley, vehicleRoundedStraightPathMaxStanley), "Generated Straight Path And Stanley Controlled Vehicle Trajectory");


    double radius = 7;
    initialX = 0;
    initialY = 0;
    // Generate and visualize a circle path, with fields: centerX, centerY, radius, dt
    // Uses P controller to follow the path
    Path circlePath(initialX, initialY, radius, vehicle.dt*3);

    double roundedCirclePathMax = simulateCirclePath(circlePath, environment);
    if (circlePath.x.size() >= 2) {
        controlledVehicleWithPredefinedCircularPath.setInitialTheta(atan2(circlePath.y[1] - circlePath.y[0], circlePath.x[1] - circlePath.x[0]));
    } else {
        controlledVehicleWithPredefinedCircularPath.setInitialTheta(0);
    }
    controlledVehicleWithPredefinedCircularPath.setInitialX(circlePath.x.front());
    controlledVehicleWithPredefinedCircularPath.setInitialY(circlePath.y.front());

    double vehicleRoundedCirclePathMax = simulatePathFollowingPAndGetRoundedMax(controlledVehicleWithPredefinedCircularPath, circlePath, environment, controller);
    runApp(controlledVehicleWithPredefinedCircularPath, circlePath, max(vehicleRoundedCirclePathMax, roundedCirclePathMax), "Generated Circle Path And Controlled Vehicle Trajectory");

     // Generate and visualize a circle path, with fields: centerX, centerY, radius, dt
     // Uses PID controller to follow the path
    Path circlePathPID(initialX, initialY, radius, vehicle.dt*3);
    resetPathPIDState(controller);
    double roundedCirclePathMaxPID = simulateCirclePath(circlePathPID, environment);
    if (circlePathPID.x.size() >= 2) {
        controlledPIDVehicleWithPredefinedCircularPath.setInitialTheta(atan2(circlePathPID.y[1] - circlePathPID.y[0], circlePathPID.x[1] - circlePathPID.x[0]));
    } else {
        controlledPIDVehicleWithPredefinedCircularPath.setInitialTheta(0);
    }
    controlledPIDVehicleWithPredefinedCircularPath.setInitialX(circlePathPID.x.front());
    controlledPIDVehicleWithPredefinedCircularPath.setInitialY(circlePathPID.y.front());

    double vehiclePIDRoundedCirclePathMax = simulatePathFollowingPIDAndGetRoundedMax(controlledPIDVehicleWithPredefinedCircularPath, circlePathPID, environment, controller);
    runApp(controlledPIDVehicleWithPredefinedCircularPath, circlePathPID, max(vehiclePIDRoundedCirclePathMax, roundedCirclePathMaxPID), "Generated Circle Path And PID Controlled Vehicle Trajectory");

    // Generate and visualize a circle path, with fields: centerX, centerY, radius, dt
     // Uses Stanley controller to follow the path
    Path circlePathStanley(initialX, initialY, radius, vehicle.dt*3);
    double roundedCirclePathMaxStanley = simulateCirclePath(circlePathStanley, environment);
    if (circlePathStanley.x.size() >= 2) {
        controlledStanleyVehicleWithPredefinedCircularPath.setInitialTheta(atan2(circlePathStanley.y[1] - circlePathStanley.y[0], circlePathStanley.x[1] - circlePathStanley.x[0]));
    } else {
        controlledStanleyVehicleWithPredefinedCircularPath.setInitialTheta(0);
    }
    controlledStanleyVehicleWithPredefinedCircularPath.setInitialX(circlePathStanley.x.front());
    controlledStanleyVehicleWithPredefinedCircularPath.setInitialY(circlePathStanley.y.front());

    double vehicleStanleyRoundedCirclePathMax = simulatePathFollowingStanleyAndGetRoundedMax(controlledStanleyVehicleWithPredefinedCircularPath, circlePathStanley, environment, controller);
    runApp(controlledStanleyVehicleWithPredefinedCircularPath, circlePathStanley, max(vehicleStanleyRoundedCirclePathMax, roundedCirclePathMaxStanley), "Generated Circle Path And Stanley Controlled Vehicle Trajectory");
    


    initialX = 0;
    initialY = 0;
    // Generate and visualize a sine path, with fields: x, y, heading, amplitude, wavelength, length (along that heading), dt
    // Uses P controller to follow the path
    Path sinePath(initialX, initialY, 0, 30, 30, 90, vehicle.dt*2);

    double roundedSinePathMax = simulateSinePath(sinePath, environment);
    if (sinePath.x.size() >= 2) {
        controlledVehicleWithPredefinedSinePath.setInitialTheta(atan2(sinePath.y[1] - sinePath.y[0], sinePath.x[1] - sinePath.x[0]));
    } else {
        controlledVehicleWithPredefinedSinePath.setInitialTheta(0);
    }
    controlledVehicleWithPredefinedSinePath.setInitialX(sinePath.x.front());
    controlledVehicleWithPredefinedSinePath.setInitialY(sinePath.y.front());

    double vehicleRoundedSinePathMax = simulatePathFollowingPAndGetRoundedMax(controlledVehicleWithPredefinedSinePath, sinePath, environment, controller);
    runApp(controlledVehicleWithPredefinedSinePath ,sinePath, max(vehicleRoundedSinePathMax, roundedSinePathMax), "Generated Sine Path And Controlled Vehicle Trajectory");

    // Generate and visualize a sine path, with fields: x, y, heading, amplitude, wavelength, length (along that heading), dt
    // Uses PID controller to follow the path
    Path sinePathPID(initialX, initialY, 0, 30, 30, 90, vehicle.dt*2);
    resetPathPIDState(controller);
    double roundedSinePathMaxPID = simulateSinePath(sinePathPID, environment);
    if (sinePathPID.x.size() >= 2) {
        controlledPIDVehicleWithPredefinedSinePath.setInitialTheta(atan2(sinePathPID.y[1] - sinePathPID.y[0], sinePathPID.x[1] - sinePathPID.x[0]));
    } else {
        controlledPIDVehicleWithPredefinedSinePath.setInitialTheta(0);
    }
    controlledPIDVehicleWithPredefinedSinePath.setInitialX(sinePathPID.x.front());
    controlledPIDVehicleWithPredefinedSinePath.setInitialY(sinePathPID.y.front());

    double vehiclePIDRoundedSinePathMax = simulatePathFollowingPIDAndGetRoundedMax(controlledPIDVehicleWithPredefinedSinePath, sinePathPID, environment, controller);
    runApp(controlledPIDVehicleWithPredefinedSinePath ,sinePathPID, max(vehiclePIDRoundedSinePathMax, roundedSinePathMaxPID), "Generated Sine Path And PID Controlled Vehicle Trajectory");

    // Generate and visualize a sine path, with fields: x, y, heading, amplitude, wavelength, length (along that heading), dt
    // Uses Stanley controller to follow the path
    Path sinePathStanley(initialX, initialY, 0, 30, 30, 90, vehicle.dt*2);
    double roundedSinePathMaxStanley = simulateSinePath(sinePathStanley, environment);
    if (sinePathStanley.x.size() >= 2) {
        controlledStanleyVehicleWithPredefinedSinePath.setInitialTheta(atan2(sinePathStanley.y[1] - sinePathStanley.y[0], sinePathStanley.x[1] - sinePathStanley.x[0]));
    } else {
        controlledStanleyVehicleWithPredefinedSinePath.setInitialTheta(0);
    }
    controlledStanleyVehicleWithPredefinedSinePath.setInitialX(sinePathStanley.x.front());
    controlledStanleyVehicleWithPredefinedSinePath.setInitialY(sinePathStanley.y.front());

    double vehicleStanleyRoundedSinePathMax = simulatePathFollowingStanleyAndGetRoundedMax(controlledStanleyVehicleWithPredefinedSinePath, sinePathStanley, environment, controller);
    runApp(controlledStanleyVehicleWithPredefinedSinePath ,sinePathStanley, max(vehicleStanleyRoundedSinePathMax, roundedSinePathMaxStanley), "Generated Sine Path And Stanley Controlled Vehicle Trajectory");



    initialX = 0;
    initialY = 0;
    // Generate and visualize a chicane turn path, with fields: x, y, heading, radius, entryLenght, middleLenght, finalLenght, dt
    // Uses PID controller to follow the path
    Path chicanePath(initialX, initialY, 0, 5, 12, 6, 12, vehicle.dt*2);
    resetPathPIDState(controller);
    double roundedSharpTurnPathMaxPID = simulateChicanePath(chicanePath, environment);
    if (chicanePath.x.size() >= 2) {
        controlledPIDVehicleWithPredefinedSharpTurnPath.setInitialTheta(atan2(chicanePath.y[1] - chicanePath.y[0], chicanePath.x[1] - chicanePath.x[0]));
    } else {
        controlledPIDVehicleWithPredefinedSharpTurnPath.setInitialTheta(0);
    }
    controlledPIDVehicleWithPredefinedSharpTurnPath.setInitialX(chicanePath.x.front());
    controlledPIDVehicleWithPredefinedSharpTurnPath.setInitialY(chicanePath.y.front());   

    double vehiclePIDRoundedSharpTurnPathMax = simulatePathFollowingPIDAndGetRoundedMax(controlledPIDVehicleWithPredefinedSharpTurnPath, chicanePath, environment, controller);
    runApp(controlledPIDVehicleWithPredefinedSharpTurnPath ,chicanePath, max(vehiclePIDRoundedSharpTurnPathMax, roundedSharpTurnPathMaxPID), "Generated Sharp Turn Path And PID Controlled Vehicle Trajectory");

    // Generate and visualize a chicane turn path, with fields: x, y, heading, radius, entryLenght, middleLenght, finalLenght, dt
    // Uses Stanely controller to follow the path
    Path chicanePathStanley(initialX, initialY, 0, 5, 12, 6, 12, vehicle.dt*2);
    double roundedSharpTurnPathMaxStanley = simulateChicanePath(chicanePathStanley, environment);
    if (chicanePathStanley.x.size() >= 2) {
        controlledStanleyVehicleWithPredefinedSharpTurnPath.setInitialTheta(atan2(chicanePathStanley.y[1] - chicanePathStanley.y[0], chicanePathStanley.x[1] - chicanePathStanley.x[0]));
    } else {
        controlledStanleyVehicleWithPredefinedSharpTurnPath.setInitialTheta(0);
    }
    controlledStanleyVehicleWithPredefinedSharpTurnPath.setInitialX(chicanePathStanley.x.front());
    controlledStanleyVehicleWithPredefinedSharpTurnPath.setInitialY(chicanePathStanley.y.front());   

    double vehicleStanleyRoundedSharpTurnPathMax = simulatePathFollowingStanleyAndGetRoundedMax(controlledStanleyVehicleWithPredefinedSharpTurnPath, chicanePathStanley, environment, controller);
    runApp(controlledStanleyVehicleWithPredefinedSharpTurnPath ,chicanePathStanley, max(vehicleStanleyRoundedSharpTurnPathMax, roundedSharpTurnPathMaxStanley), "Generated Sharp Turn Path And Stanley Controlled Vehicle Trajectory");

    return 0;
}


