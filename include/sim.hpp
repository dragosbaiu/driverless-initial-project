#ifndef SIM_HPP
#define SIM_HPP

class Vehicle;
class Path;
struct Controller;
struct Environment;

double simulateBase(Vehicle& vehicle, Environment& environment);
double simulateBaseDrift(Vehicle& vehicle, Environment& environment);
double simulateHeadingStraight(Vehicle& vehicle, Environment& environment, Controller& controller);
double simulateHeadingStraightDrift(Vehicle& vehicle, Environment& environment, Controller& controller);

double simulateStraightPath(Path& path, Environment& environment);
double simulateCircularPath(Path& path, Environment& environment);
double simulateSinePath(Path& path, Environment& environment);

double simulateP(Vehicle& vehicle, Path& path, Environment& environment, Controller& controller); 
double simulatePID(Vehicle& vehicle, Path& path, Environment& environment, Controller& controller);
double simulateChicanePath(Path& path, Environment& environment);
double simulateStanley(Vehicle& vehicle, Path& path, Environment& environment, Controller& controller);

#endif