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

    // Create copies of the vehicle for different simulation scenarios
    Vehicle standardVehicle = vehicle;
    Vehicle standardVehicleWithDrift = vehicle;
    Vehicle controlledVehicle = vehicle;
    Vehicle controlledVehicleWithDrift = vehicle;

    // Simulate and visualize standard bicycle model
    double roundedStandardMax = simulateAndGetRoundedMax(standardVehicle, environment);
    runApp(standardVehicle, roundedStandardMax, "Trajectory of Standard Bicycle Model");

    // Simulate and visualize bicycle model with noise and lateral drift
    double roundedDriftMax = simulateWithDriftAndGetRoundedMax(standardVehicleWithDrift, environment);
    runApp(standardVehicleWithDrift, roundedDriftMax, "Trajectory of Bicycle with Noise and Lateral Drift");

    // Simulate and visualize controlled bicycle model
    double roundedControlledMax = simulateStraightPathAndGetRoundedMax(controlledVehicle, environment, controller);
    runApp(controlledVehicle, roundedControlledMax, "Trajectory of Controlled Bicycle");

    // Simulate and visualize controlled bicycle model with noise and lateral drift
    double roundedControlledDriftMax = simulateStraightPathWithDriftAndGetRoundedMax(controlledVehicleWithDrift, environment, controller);
    runApp(controlledVehicleWithDrift, roundedControlledDriftMax, "Trajectory of Controlled Bicycle with Noise and Lateral Drift");

    return 0;
}



