#include "io.hpp"
#include "vehicle.hpp"
#include <fstream>
#include <iostream>

using namespace std;

bool read_input(const string& path, Vehicle& vehicle){
    fstream file (path);
    if (!file){
        cerr << "Unable to open file " << path << '\n';
        return false;
    }

    file >> vehicle.x[0] >> vehicle.y[0] >> vehicle.theta[0] >> vehicle.velocity[0];
    file >> vehicle.acceleration >> vehicle.delta >> vehicle.dt >> vehicle.L;

    if (!file){
        cerr << "Input format error in " << path << '\n';
        return false;
    }

    file.close();
    return true;
}