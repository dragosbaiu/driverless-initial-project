#include "vehicle.hpp"
#include "io.hpp"
#include "sim.hpp"
#include "graphics.hpp"
#include "control.hpp"
#include "noise.hpp"
#include <iostream>

using namespace std;

int main() {
    
    Vehicle vehicle;

    if (!readInput("data/input.txt", vehicle)) {
        return 1;
    }

    Vehicle standardVehicle = vehicle;
    Vehicle standardVehicleWithDrift = vehicle;
    Vehicle controlledVehicle = vehicle;
    Vehicle controlledVehicleWithDrift = vehicle;

    double roundedStandardMax = simulateAndGetRoundedMax(standardVehicle, 200);
    runApp(standardVehicle, roundedStandardMax, "Trajectory of Standard Bicycle Model");

    double roundedDriftMax = simulateWithDriftAndGetRoundedMax(standardVehicleWithDrift, 200);
    runApp(standardVehicleWithDrift, roundedDriftMax, "Trajectory of Bicycle with Lateral Drift");

    double roundedControlledMax = simulateStraightPathAndGetRoundedMax(controlledVehicle, 200);
    runApp(controlledVehicle, roundedControlledMax, "Trajectory of Controlled Bicycle");

    double roundedControlledDriftMax = simulateStraightPathWithDriftAndGetRoundedMax(controlledVehicleWithDrift, 200);
    runApp(controlledVehicleWithDrift, roundedControlledDriftMax, "Trajectory of Controlled Bicycle with Lateral Drift");

    return 0;
}



