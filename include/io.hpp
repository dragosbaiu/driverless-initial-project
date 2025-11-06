#ifndef IO_HPP
#define IO_HPP

#include <string>

class Vehicle;
struct Controller;
struct Environment;

bool readInput(const std::string& path, Vehicle& vehicle, Controller& controller, Environment& environment);

#endif