#ifndef SIM_HPP
#define SIM_HPP

class Vehicle;
struct Controller;
struct Environment;

double simulateAndGetRoundedMax(Vehicle& vehicle, Environment& environment);
double simulateWithDriftAndGetRoundedMax(Vehicle& vehicle, Environment& environment);
double simulateStraightPathAndGetRoundedMax(Vehicle& vehicle, Environment& environment, Controller& controller);
double simulateStraightPathWithDriftAndGetRoundedMax(Vehicle& vehicle, Environment& environment, Controller& controller);

#endif