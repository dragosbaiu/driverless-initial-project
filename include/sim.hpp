#ifndef SIM_HPP
#define SIM_HPP

class Vehicle;

double simulateAndGetRoundedMax(Vehicle& vehicle, int steps);
double simulateStraightPathAndGetRoundedMax(Vehicle& vehicle, int steps);

#endif