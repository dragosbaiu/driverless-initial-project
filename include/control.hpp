#ifndef CONTROL_HPP
#define CONTROL_HPP

class Vehicle;
struct Environment;
struct Controller;
class Path;

double computeHeadingCorrection(Controller& controller, double targetHeadingAngle, double currentHeadingAngle);
double computeDriftError(Vehicle& vehicle);
double computeHeadingAndDriftCorrection(Vehicle& vehicle, Environment& environment, Controller& controller, double targetHeadingAngle, double currentHeadingAngle);
double computeCrossTrackError(Vehicle& vehicle, Path& path);
double computeSteeringForPathFollowing(Vehicle& vehicle, Path& path, Controller& controller);
#endif
