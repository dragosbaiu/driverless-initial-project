#ifndef SIM_HPP
#define SIM_HPP

class Vehicle;
class Path;
struct Controller;
struct Environment;

// Simulation functions to simulate vehicle movement and return rounded maximum distance from origin

double simulateAndGetRoundedMax(Vehicle& vehicle, Environment& environment);
double simulateWithDriftAndGetRoundedMax(Vehicle& vehicle, Environment& environment);
double simulateStraightPathAndGetRoundedMax(Vehicle& vehicle, Environment& environment, Controller& controller);
double simulateStraightPathWithDriftAndGetRoundedMax(Vehicle& vehicle, Environment& environment, Controller& controller);

double simulateStraightPath(Path& path, Environment& environment);
double simulateCirclePath(Path& path, Environment& environment);
double simulateSinePath(Path& path, Environment& environment);

#endif