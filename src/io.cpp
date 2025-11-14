#include "io.hpp"
#include "vehicle.hpp"
#include "controller.hpp"
#include "environment.hpp"
#include "path.hpp"
#include <fstream>
#include <iostream>

using namespace std;

// Read vehicle, controller, and environment parameters from input file
bool readInput(const string& filePath, Vehicle& vehicle, Controller& controller, Environment& environment) {
    fstream file (filePath);
    if (!file){
        cerr << "Unable to open file " << filePath << '\n';
        return false;
    }

    file >> vehicle.x[0] >> vehicle.y[0] >> vehicle.theta[0] >> vehicle.velocity[0];
    file >> vehicle.acceleration >> vehicle.delta >> vehicle.dt >> vehicle.L;

    file >> controller.Kp >> controller.maxSteeringAngle >> environment.lateralDrift;
    file >> controller.Kpx >> environment.steps >> environment.stepsPathFollowing >> controller.mean >> controller.standardDeviation;
    file >> controller.Ki >> controller.Kd;
    file >> controller.Ks;

    if (!file){
        cerr << "Input format error in " << filePath << '\n';
        return false;
    }

    file.close();
    return true;
}