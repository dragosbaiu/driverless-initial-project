#include <iostream>
#include <cmath>
#include "vehicle.hpp"

using namespace std;

Vehicle::Vehicle() 
    : acceleration(0), delta(0), dt(0), L(0) 
    {
        x.push_back(0);
        y.push_back(0);
        theta.push_back(0);
        velocity.push_back(0);
    }
    
Vehicle::Vehicle(double x, double y, double theta, double velocity, double acceleration, double delta, double dt, double L)
    : acceleration(acceleration), delta(delta), dt(dt), L(L) 
    {
        this->x.push_back(x);
        this->y.push_back(y);
        this->theta.push_back(theta);
        this->velocity.push_back(velocity);
    }

void Vehicle::updatePosition(){
    velocity.push_back(velocity.back() + acceleration * dt);
    theta.push_back(theta.back() + (velocity.back() / L) * tan(delta) * dt);
    x.push_back(x.back() + velocity.back() * cos(theta.back()) * dt);
    y.push_back(y.back() + velocity.back() * sin(theta.back()) * dt);
}
