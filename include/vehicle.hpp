#ifndef VEHICLE_HPP
#define VEHICLE_HPP

#include <vector>

class Vehicle {
    public:
        std::vector <double> x, y, theta, velocity;
        double acceleration, delta, dt, L;
        Vehicle(double x, double y, double theta, double velocity, double acceleration, double delta, double dt, double L);
        Vehicle();
        void updatePosition();
};

#endif