#include "io.hpp"
#include "vehicle.hpp"
#include "controller.hpp"
#include "environment.hpp"
#include <fstream>
#include <iostream>

using namespace std;

// Read vehicle, controller, and environment parameters from input file
bool readInput(const string& path, Vehicle& vehicle, Controller& controller, Environment& environment) {
    fstream file (path);
    if (!file){
        cerr << "Unable to open file " << path << '\n';
        return false;
    }

    file >> vehicle.x[0] >> vehicle.y[0] >> vehicle.theta[0] >> vehicle.velocity[0];
    file >> vehicle.acceleration >> vehicle.delta >> vehicle.dt >> vehicle.L;

    file >> controller.Kp >> controller.maxSteeringAngle >> environment.lateralDrift;
    file >> controller.Ky >> environment.steps ;

    if (!file){
        cerr << "Input format error in " << path << '\n';
        return false;
    }

    file.close();
    return true;
}