#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

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

    double Ks;
};


struct PathError {
    double crossTrackError;
    double pathHeading;
};

#endif