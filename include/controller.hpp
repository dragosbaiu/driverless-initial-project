#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

// Controller structure to hold control parameters (Kp - proportional gain, Kpx - drift gain, maxSteeringAngle)
struct Controller{
    double Kp;
    double Kpx;
    double maxSteeringAngle;
    double mean;
    double standardDeviation;
};

#endif