#ifndef CONTROL_HPP
#define CONTROL_HPP

class Vehicle;
struct Environment;
struct Controller;
class Path;
struct PathError;

double steerHeading(Controller& controller, double targetHeadingAngle, double currentHeadingAngle);
double computeDriftError(Vehicle& vehicle);
double steerHeadingDrift(Vehicle& vehicle, Environment& environment, Controller& controller, double targetHeadingAngle, double currentHeadingAngle);

double computeCrossTrackError(Vehicle& vehicle, Path& path);
double steerHeadingP(Vehicle& vehicle, Path& path, Controller& controller);

double steerHeadingPID(Vehicle& vehicle, Path& path, Controller& controller);
void resetPathPIDState(Controller& controller);

PathError computeCrossTrackErrorAndPathHeading(Vehicle& vehicle, Path& path);
double normalizeAngle(double angle);
double steerHeadingStanley(Vehicle& vehicle, Path& path, Controller& controller);

#endif
