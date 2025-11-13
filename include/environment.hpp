#ifndef ENVIRONMENT_HPP
#define ENVIRONMENT_HPP

// Environment structure to hold simulation environment parameters (lateralDrift - lateral drift value, steps - number of simulation steps)
struct Environment{
    double lateralDrift;
    int steps;
    int stepsPathFollowing;
};

#endif