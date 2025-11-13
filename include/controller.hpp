#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

// Controller structure to hold control parameters (Kp - proportional gain, Kpx - drift gain, maxSteeringAngle)
struct Controller{
    double Kp;
    double Kpx;

    double maxSteeringAngle;
    double mean;
    double standardDeviation;

    double Ki;  
    double Kd;

    double integralError = 0.0;
    double previousError = 0.0;
    bool firstUpdate = true;
};

#endif