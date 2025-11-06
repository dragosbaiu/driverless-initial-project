#ifndef CONTROL_HPP
#define CONTROL_HPP

#include "vehicle.hpp"

double proportionalControl(double targetHeadingAngle, double currentHeadingAngle, double Kp, double maxSteeringAngle);
double computeCrossTrackError(const Vehicle& vehicle);
double proportionalControlWithLateralDrift(const Vehicle& vehicle, double targetHeadingAngle, double currentHeadingAngle, double Kp, double maxSteeringAngle, double lateralDrift, double Ky);

#endif
