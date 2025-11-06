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

    double roundedStandardMax = simulateAndGetRoundedMax(standardVehicle, 100);
    runApp(standardVehicle, roundedStandardMax);

    double roundedNoisyMax = simulateStraightPathAndGetRoundedMax(noisyVehicle, 100);
    runApp(noisyVehicle, roundedNoisyMax);

    return 0;
}



