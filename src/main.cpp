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
    Vehicle noisyVehicle = vehicle;
    Vehicle standardVehicleWithDrift = vehicle;
    Vehicle noisyVehicleWithDrift = vehicle;

    double roundedStandardMax = simulateAndGetRoundedMax(standardVehicle, 200);
    runApp(standardVehicle, roundedStandardMax);

    double roundedDriftMax = simulateWithDriftAndGetRoundedMax(standardVehicleWithDrift, 200);
    runApp(standardVehicleWithDrift, roundedDriftMax);

    double roundedNoisyMax = simulateStraightPathAndGetRoundedMax(noisyVehicle, 200);
    runApp(noisyVehicle, roundedNoisyMax);

    double roundedNoisyDriftMax = simulateStraightPathWithDriftAndGetRoundedMax(noisyVehicleWithDrift, 200);
    runApp(noisyVehicleWithDrift, roundedNoisyDriftMax);

    return 0;
}



