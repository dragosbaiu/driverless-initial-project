#ifndef PATH_HPP
#define PATH_HPP

#include <vector>

class Path {
    public:
        std::vector <double> x, y, theta;
        double delta, dt, length, traveled;
        double centerX, centerY, radius;
        double amplitude, wavelength;
        double entryStraight, middleStraight, exitStraight;
        Path();
        Path(double x, double y, double theta, double length, double dt);
        Path(double centerX, double centerY, double radius, double dt);
        Path(double x, double y, double theta, double amplitude, double wavelength, double length, double dt);
        Path(double x, double y, double theta, double radius, double entryStraight, double middleStraight, double exitStraight, double dt);
        void generateStraightPath();
        void generateCirclePath();
        void generateSinePath();
        void generateChicanePath();

};

#endif