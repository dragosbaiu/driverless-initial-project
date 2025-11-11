#ifndef IO_HPP
#define IO_HPP

#include <string>

class Vehicle;
class Path;
struct Controller;
struct Environment;

bool readInput(const std::string& filePath, Vehicle& vehicle, Controller& controller, Environment& environment);

#endif