#include "vehicle.hpp"
#include "io.hpp"
#include "sim.hpp"
#include "graphics.hpp"
#include "control.hpp"
#include "noise.hpp"
#include "controller.hpp"
#include "environment.hpp"
#include <iostream>

using namespace std;

int main() {
    
    Vehicle vehicle;
    Controller controller;
    Environment environment;

    if (!readInput("data/input.txt", vehicle, controller, environment)) {
        return 1;
    }

    Vehicle standardVehicle = vehicle;
    Vehicle standardVehicleWithDrift = vehicle;
    Vehicle controlledVehicle = vehicle;
    Vehicle controlledVehicleWithDrift = vehicle;

    double roundedStandardMax = simulateAndGetRoundedMax(standardVehicle, environment);
    runApp(standardVehicle, roundedStandardMax, "Trajectory of Standard Bicycle Model");

    double roundedDriftMax = simulateWithDriftAndGetRoundedMax(standardVehicleWithDrift, environment);
    runApp(standardVehicleWithDrift, roundedDriftMax, "Trajectory of Bicycle with Lateral Drift");

    double roundedControlledMax = simulateStraightPathAndGetRoundedMax(controlledVehicle, environment, controller);
    runApp(controlledVehicle, roundedControlledMax, "Trajectory of Controlled Bicycle");

    double roundedControlledDriftMax = simulateStraightPathWithDriftAndGetRoundedMax(controlledVehicleWithDrift, environment, controller);
    runApp(controlledVehicleWithDrift, roundedControlledDriftMax, "Trajectory of Controlled Bicycle with Lateral Drift");

    return 0;
}



