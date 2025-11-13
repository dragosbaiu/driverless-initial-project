#include <iostream>
#include <cmath>
#include "vehicle.hpp"
#include "environment.hpp"

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

// Update vehicle position based on kinematic bicycle model
void Vehicle::updatePosition(){
    velocity.push_back(velocity.back() + acceleration * dt);
    theta.push_back(theta.back() + (velocity.back() / L) * tan(delta) * dt);
    x.push_back(x.back() + velocity.back() * cos(theta.back()) * dt);
    y.push_back(y.back() + velocity.back() * sin(theta.back()) * dt);
}

// Apply lateral drift to the vehicle's position
void Vehicle::applyLateralDrift(Environment& environment){
    x.back() = x.back() - environment.lateralDrift * sin(theta.back()) * dt;
    y.back() = y.back() + environment.lateralDrift * cos(theta.back()) * dt;
}

void Vehicle::setInitialX(double newX){
    x[0] = newX;
}

void Vehicle::setInitialY(double newY){
    y[0] = newY;
}

void Vehicle::setInitialTheta(double newTheta){
    theta[0] = newTheta;
}