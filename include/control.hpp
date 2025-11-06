#ifndef CONTROL_HPP
#define CONTROL_HPP

class Vehicle;
struct Environment;
struct Controller;

double proportionalControl(Controller& controller, double targetHeadingAngle, double currentHeadingAngle);
double computeDriftError(Vehicle& vehicle);
double proportionalControlWithLateralDrift(Vehicle& vehicle, Environment& environment, Controller& controller, double targetHeadingAngle, double currentHeadingAngle);
#endif
