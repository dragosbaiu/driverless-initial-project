#include "vehicle.hpp"
#include "io.hpp"
#include "sim.hpp"
#include "graphics.hpp"
#include "control.hpp"
#include "noise.hpp"
#include "controller.hpp"
#include "environment.hpp"
#include "path.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

using namespace std;

int main() {
    
    Vehicle vehicle;
    Controller controller;
    Environment environment;

    if (!readInput("data/input.txt", vehicle, controller, environment)) {
        return 1;
    }

    double pathRoundedMax;
    double roundedMax;

    // Simulate and visualize standard bicycle model
    Vehicle base = vehicle;
    roundedMax = simulateBase(base, environment);
    runApp(base, roundedMax, "Trajectory of Standard Bicycle Model");

    // Simulate and visualize bicycle model with lateral drift
    Vehicle baseDrift = vehicle;
    roundedMax = simulateBaseDrift(baseDrift, environment);
    runApp(baseDrift, roundedMax, "Trajectory of Standard Bicycle Model with Lateral Drift");

    // Simulate and visualize bicycle model controlled to have a straight heading
    Vehicle headingStraight = vehicle;
    roundedMax = simulateHeadingStraight(headingStraight, environment, controller);
    runApp(headingStraight, roundedMax, "Trajectory of Bicycle Controlled to Have a Straight Heading");

    // Simulate and visualize bicycle model controlled to have a straight heading, with noise and lateral drift
    Vehicle headingStraightDrift = vehicle;
    roundedMax = simulateHeadingStraightDrift(headingStraightDrift, environment, controller);
    runApp(headingStraightDrift, roundedMax, "Trajectory of Bicycle Controlled to Have a Straight Heading, with Noise and Lateral Drift");


    // Generate and visualize a straight path   
    // Path Fields: x, y, heading, lenght, time step
    double initialX = 0;
    double initialY = 0;
    Path straightPath(initialX, initialY, 1, 100, vehicle.dt*1.5);
    pathRoundedMax = simulateStraightPath(straightPath, environment);

    // P controller for following the path
    Vehicle pStraight = vehicle;
    pStraight.setInitialTheta(straightPath);
    pStraight.setInitialX(straightPath.x.front());
    pStraight.setInitialY(straightPath.y.front());
    roundedMax = simulateP(pStraight, straightPath, environment, controller);
    runApp(pStraight, straightPath, max (pathRoundedMax, roundedMax), "Straight Trajectory of Bicycle: P Controller");

    // PID controller for following the path
    Vehicle pidStraight = vehicle;
    resetPathPIDState(controller);
    pidStraight.setInitialTheta(straightPath);
    pidStraight.setInitialX(straightPath.x.front());
    pidStraight.setInitialY(straightPath.y.front());
    roundedMax = simulatePID(pidStraight, straightPath, environment, controller);
    runApp(pidStraight, straightPath, max (pathRoundedMax, roundedMax), "Straight Trajectory of Bicycle: PID Controller");

    // Stanley controller for following the path
    Vehicle stanleyStraight = vehicle;
    stanleyStraight.setInitialTheta(straightPath);
    stanleyStraight.setInitialX(straightPath.x.front());
    stanleyStraight.setInitialY(straightPath.y.front());
    roundedMax = simulateStanley(stanleyStraight, straightPath, environment, controller);
    runApp(stanleyStraight, straightPath, max (pathRoundedMax, roundedMax), "Straight Trajectory of Bicycle: Stanley Controller");


    // Generate and visualize a circular path
     // Path Fields: centerX, centerY, radius, time step
    double radius = 7;
    initialX = 0;
    initialY = 0;
    Path circularPath(initialX, initialY, radius, vehicle.dt*3);
    pathRoundedMax = simulateCircularPath(circularPath, environment);

    // P controller for following the path
    Vehicle pCircular = vehicle;
    pCircular.setInitialTheta(circularPath);
    pCircular.setInitialX(circularPath.x.front());
    pCircular.setInitialY(circularPath.y.front());
    roundedMax = simulateP(pCircular, circularPath, environment, controller);
    runApp(pCircular, circularPath, max(roundedMax, pathRoundedMax), "Circular Trajectory of Bicycle: P Controller");

    // PID controller for following the path
    Vehicle pidCircular = vehicle;
    resetPathPIDState(controller);
    pidCircular.setInitialTheta(circularPath);
    pidCircular.setInitialX(circularPath.x.front());
    pidCircular.setInitialY(circularPath.y.front());
    roundedMax = simulatePID(pidCircular, circularPath, environment, controller);
    runApp(pidCircular, circularPath, max(roundedMax, pathRoundedMax), "Circular Trajectory of Bicycle: PID Controller");

    // Stanley controller for following the path
    Vehicle stanleyCircular = vehicle;
    stanleyCircular.setInitialTheta(circularPath);
    stanleyCircular.setInitialX(circularPath.x.front());
    stanleyCircular.setInitialY(circularPath.y.front());
    roundedMax = simulateStanley(stanleyCircular, circularPath, environment, controller);
    runApp(stanleyCircular, circularPath, max(roundedMax, pathRoundedMax), "Circular Trajectory of Bicycle: Stanley Controller");
    

    // Generate and visualize a Sine path
    // Fields: x, y, heading, amplitude, wavelength, length (along that heading), time step
    initialX = 0;
    initialY = 0;
    Path sinePath(initialX, initialY, 0, 30, 30, 90, vehicle.dt*2);
    pathRoundedMax = simulateSinePath(sinePath, environment);

    // P controller for following the path
    Vehicle pSine = vehicle;
    pSine.setInitialTheta(sinePath);
    pSine.setInitialX(sinePath.x.front());
    pSine.setInitialY(sinePath.y.front());
    roundedMax = simulateP(pSine, sinePath, environment, controller);
    runApp(pSine ,sinePath, max(roundedMax, pathRoundedMax), "Sine Trajectory of Bicycle: P Controller");

    // PID controller for following the path
    Vehicle pidSine = vehicle;
    resetPathPIDState(controller);
    pidSine.setInitialTheta(sinePath);
    pidSine.setInitialX(sinePath.x.front());
    pidSine.setInitialY(sinePath.y.front());
    roundedMax = simulatePID(pidSine, sinePath, environment, controller);
    runApp(pidSine ,sinePath, max(roundedMax, pathRoundedMax), "Sine Trajectory of Bicycle: PID Controller");

    // Stanley controller for following the path
    Vehicle stanleySine = vehicle;
    stanleySine.setInitialTheta(sinePath);
    stanleySine.setInitialX(sinePath.x.front());
    stanleySine.setInitialY(sinePath.y.front());
    roundedMax = simulateStanley(stanleySine, sinePath, environment, controller);
    runApp(stanleySine ,sinePath, max(roundedMax, pathRoundedMax), "Sine Trajectory of Bicycle: Stanley Controller");


    // Generate and visualize a chicane path
    // Fields: x, y, heading, radius, entryLenght (before first turn), middleLenght (between turns), finalLenght (after second turn), time step
    initialX = 0;
    initialY = 0;
    Path chicanePath(initialX, initialY, 0, 5, 12, 6, 12, vehicle.dt*2);
    pathRoundedMax = simulateChicanePath(chicanePath, environment);

    // Uses PID controller for following the path
    Vehicle pidChicane = vehicle;
    resetPathPIDState(controller);
    pidChicane.setInitialTheta(chicanePath);
    pidChicane.setInitialX(chicanePath.x.front());
    pidChicane.setInitialY(chicanePath.y.front());   
    roundedMax = simulatePID(pidChicane, chicanePath, environment, controller);
    runApp(pidChicane ,chicanePath, max(roundedMax, pathRoundedMax), "Chicane Trajectory of Bicycle: PID Controller");

    // Uses Stanley controller for following the path
    Vehicle stanleyChicane = vehicle;
    stanleyChicane.setInitialTheta(chicanePath);
    stanleyChicane.setInitialX(chicanePath.x.front());
    stanleyChicane.setInitialY(chicanePath.y.front());   
    roundedMax = simulateStanley(stanleyChicane, chicanePath, environment, controller);
    runApp(stanleyChicane ,chicanePath, max(roundedMax, pathRoundedMax), "Chicane Trajectory of Bicycle: Stanley Controller");

    return 0;
}


