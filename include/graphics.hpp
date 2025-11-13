#ifndef GRAPHICS_HPP
#define GRAPHICS_HPP

#include <string>

class Vehicle;
class Path;

void runApp(Vehicle& vehicle, double roundedMax, std::string windowTitle);
void runApp(Vehicle& vehicle, Path& path, double roundedMax, std::string windowTitle);

#endif