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

    // Simulate and visualize bicycle model with lateral drift
    double roundedDriftMax = simulateWithDriftAndGetRoundedMax(standardVehicleWithDrift, environment);
    runApp(standardVehicleWithDrift, roundedDriftMax, "Trajectory of Bicycle with Lateral Drift");

    // Simulate and visualize controlled bicycle model
    double roundedControlledMax = simulateStraightPathAndGetRoundedMax(controlledVehicle, environment, controller);
    runApp(controlledVehicle, roundedControlledMax, "Trajectory of Controlled Bicycle");

    // Simulate and visualize controlled bicycle model with noise and lateral drift
    double roundedControlledDriftMax = simulateStraightPathWithDriftAndGetRoundedMax(controlledVehicleWithDrift, environment, controller);
    runApp(controlledVehicleWithDrift, roundedControlledDriftMax, "Trajectory of Controlled Bicycle with Noise and Lateral Drift");

    // Generate and visualize a straight path, with fields: x, y, theta, lenght, dt
    Path path(1, 1, 1, 5, vehicle.dt);
    double roundedStraightPathMax = simulateStraightPath(path, environment);
    runApp(path, roundedStraightPathMax, "Generated Straight Path");

    // Generate and visualize a circle path, with fields: centerX, centerY, radius, dt
    Path circlePath(0, 0, 10, vehicle.dt);
    double roundedCirclePathMax = simulateCirclePath(circlePath, environment);
    runApp(circlePath, roundedCirclePathMax, "Generated Circle Path");

    // Generate and visualize a sine path, with fields: x, y, heading, amplitude, wavelength, length (along that heading), dt
    Path sinePath(0, 0, 1, 10, 20, 50, vehicle.dt);
    double roundedSinePathMax = simulateSinePath(sinePath, environment);
    runApp(sinePath, roundedSinePathMax, "Generated Sine Path");

    return 0;
}



