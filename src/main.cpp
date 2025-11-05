#include "vehicle.hpp"
#include "io.hpp"
#include "sim.hpp"
#include "graphics.hpp"
#include <iostream>

using namespace std;

int main() {
    
    Vehicle vehicle;

    if (!read_input("data/input.txt", vehicle)) {
        return 1;
    }

    double roundedMax = simulate_and_get_rounded_max(vehicle, 100);

    run_app(vehicle, roundedMax);

    return 0;
}



