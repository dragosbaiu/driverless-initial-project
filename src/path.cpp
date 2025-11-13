#include <cmath>
#include <algorithm> 
#include "path.hpp"

using namespace std;

// Default constructor
Path::Path() 
    : length(0), delta(0), dt(0.1), traveled(0)
{
    x.push_back(0);
    y.push_back(0);
    theta.push_back(0);
}

// Constructor for straight path
Path::Path(double x, double y, double theta, double length, double dt)
    : length(length), dt(dt), traveled(0)
{
    this->x.push_back(x);
    this->y.push_back(y);
    this->theta.push_back(theta);
}

// Constructor for circular path
Path::Path(double centerX, double centerY, double radius, double dt)
    : length(2 * M_PI * radius), dt(dt), traveled(0), radius(radius), centerX(centerX), centerY(centerY)
{
    this->x.push_back(centerX + radius);
    this->y.push_back(centerY);
    this->theta.push_back(0);
}

// Constructor for sine path
Path::Path(double x, double y, double theta, double amplitude, double wavelength, double length, double dt)
    : amplitude(amplitude), wavelength(wavelength), length(length), dt(dt)
{
    this->x.push_back(x);
    this->y.push_back(y);
    this->theta.push_back(theta);
}

// Generate a straight path
void Path::generateStraightPath() {

    double heading = theta.back();

    int steps = static_cast<int>(length / dt);

    for (int i = 0; i < steps; ++i) {
        x.push_back(x.back() + cos(heading) * dt);
        y.push_back(y.back() + sin(heading) * dt);
    }
}

// Generate a circular path
void Path::generateCirclePath() {
        
    const int steps = static_cast<int>((2 * M_PI) / (dt / radius));

    for (int i = 1; i <= steps; ++i) {
        x.push_back(centerX + radius * cos(i * (dt / radius)));
        y.push_back(centerY + radius * sin(i * (dt / radius)));
    }

}

// Generate a sine wave path
void Path::generateSinePath() {
    
    const int steps = static_cast<int>(length / dt);

    double x0 = x.back();
    double y0 = y.back();
    double heading = theta.back();

    for (int i = 0; i <= steps; ++i) {
        x.push_back(x0 + i * dt * cos(heading) - amplitude * sin((2.0 * M_PI / wavelength) * i * dt) * sin(heading));
        y.push_back(y0 + i * dt * sin(heading) + amplitude * sin((2.0 * M_PI / wavelength) * i * dt) * cos(heading));
    }
}