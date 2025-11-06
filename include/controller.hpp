#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

// Controller structure to hold control parameters (Kp - proportional gain, Ky - drift gain, maxSteeringAngle)
struct Controller{
    double Kp;
    double Ky;
    double maxSteeringAngle;
    double mean;
    double standardDeviation;
};

#endif