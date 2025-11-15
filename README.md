# Driverless Control Project

## Description
The project aims to implement a feedback controller for a Formula Student Driverless vehicle, using a kinematic bicycle model. 
Paths in different forms are generated (straight, circular, sinusoidal, chicane), and then different controllers are tested (Proportional, PID, Stanley).

---

## Project Structure
```bash
DRIVERLESS-INITIAL-PROJECT/
├── .vscode/                     # VSCode settings
│
├── build/                       # CMake build directory (auto-generated)
│   ├── .cmake/
│   ├── CMakeFiles/
│   ├── cmake_install.cmake
│   ├── CMakeCache.txt
│   ├── compile_commands.json
│   └── driverless-initial-project
│
├── data/                        # Input files for simulation
│   ├── input.txt
│   └── inputFields.txt
│
├── docs/                        # Pictures of simulations
│
├── include/                     # Header files
│   ├── control.hpp
│   ├── controller.hpp
│   ├── environment.hpp
│   ├── graphics.hpp
│   ├── io.hpp
│   ├── noise.hpp
│   ├── path.hpp
│   ├── sim.hpp
│   └── vehicle.hpp
│
├── src/                         # Source files
│   ├── control.cpp
│   ├── graphics.cpp
│   ├── io.cpp
│   ├── main.cpp
│   ├── noise.cpp
│   ├── path.cpp
│   ├── sim.cpp
│   └── vehicle.cpp
│
├── .gitignore                   # Git ignore rules
└── CMakeLists.txt               # Build configuration 
```
## Installation & Build Instructions

### Requirements
- C++17 Compiler
- CMake
- Raylib

### Build
```bash
cd projectRooot
mkdir build
cd build
cmake ..
make
cd projectRoot
./build/driverless-initial-project
```
---
## Usage

### Configurate parameters
You can configure parameters inside
```bash
data/input.txt
```
You can view what each parameter is inside
```bash
data/inputFields.txt
```

### Output Windows
- Trajectory of Standard Vehicle using Kinematic Bycicle Model (Task 1)
- Trajectory of Standard Vehicle with Lateral Drift (Task 2)
- Trajectory of Vehicle, Controlled to Have a Straight Heading (Task 2)
- Trajectory of Vehicle, Controlled to Have a Straight Heading, with Noise and Lateral Drift (Task 2)
- Straight Path and Trajectory of Vehicle Using P Controller (Task 3)
- Straight Path and Trajectory of Vehicle Using PID Controller (Task 4)
- Straight Path and Trajectory of Vehicle Using Stanley Controller (Task 5)
- Circular Path and Trajectory of Vehicle Using P Controller (Task 3)
- Circular Path and Trajectory of Vehicle Using PID Controller (Task 4)
- Circular Path and Trajectory of Vehicle Using Stanley Controller (Task 5)
- Sine Path and Trajectory of Vehicle Using P Controller (Task 3)
- Sine Path and Trajectory of Vehicle Using PID Controller (Task 4)
- Sine Path and Trajectory of Vehicle Using Stanley Controller (Task 5)
- Chicane Path and Trajectory of Vehicle Using PID Controller (Task 4)
- Chicane Path and Trajectory of Vehicle Using Stanley Controller (Task 5)

---

## Features

### Vehicle
- Full kinematic bicycle model  
- Lateral drift simulation  
- Gaussian noise in heading angle

### Path Generation
- Straight path  
- Circle path  
- Sine path  
- Chicane Path

### Controllers
- Straight Path Controller
- P controller  
- PID controller  
- Stanley controller 

### Visualization
- Real-time Raylib 2D rendering  
- Scaled coordinate axes and grid
- Path drawing
- Vehicle Trajectory drawing

---

## Included Mathematics

### Kinematic Bicycle Model

\[
x_{t+1} = x_t + v \cos(\theta_t)\,dt
\]

\[
y_{t+1} = y_t + v \sin(\theta_t)\,dt
\]

\[
\theta_{t+1} = \theta_t + \frac{v}{L}\tan(\delta_t)\,dt
\]

### Controller Formulas

P Controller

\[
\delta = K_p \cdot e
\]

PID Controller

\[
\delta = K_p e + K_d \frac{de}{dt} + K_i \int e \, dt
\]

Stanley Controller

\[
\delta = \theta_e + \tan^{-1}\left(\frac{K e}{v}\right)
\]

### Cross-Track Error (CTE)
Computed as the perpendicular distance from the vehicle to the nearest point on the path.



---

# Short Report

## Approach Used

The project follows a modular design, with different classes: Vehicle, Path, Environment (for Lateral Drift and Simulation Steps), and Controller (for the controller parameters). 
At first, the kynematic bycicle equations are implemented, and then trajectories are plotted . To create an accurante vizualization, the maximum coordinate reached by the vehicle 
(either on x or y) is taken, and then the axes and grids are scaled accordingly. For the latter tasks, paths of different shapes are generated. The inital coordinate and heading
are decided by the user, followed by specific parameters for each type of path (radius for circular paths, wavelength and amplitude for sine path, etc.). Then, the simulation takes place,
in which the vehicle uses a specific controller to update its stearing angle. Moreover, inside the simulation, the vehicle updates its position after each time step. For some simulations, 
lateral drift is applied during the simulatin process to view its effects on the trajectory of the vehicle. Nevertheless, in some cases noise is also applied (in this project Gaussian Noise 
was used due to its simple implementation). The output displays the trajectory of the vehicle with yellow, and the path with blue (if present).

---

## Controller Tuning Strategy

For the Proportional Controller, the approach used to tune the Proportional (Kp) was to start with small values (~0.1) and increase it slowly until the model starts to oscillate.
For the PID controller, the approach was the same: At beggining, Kp was set to a low value, and the Derivative Gain (Kd) and Integral Gain (Ki) were set to 0. As osscilations appeared, 
Kd was increased slightly, to reduce them. However, after a certain point, the turns became less agressive, making the vehicle unable to follow sharp turns. Following this, Ki was increased,
starting with very low values (0.005, 0.01) and ending up much lower in comparison to Kd and Kp.
Finally, for the Stanley controller, Stanley gain started out at a low value (0.1), and then it was increased until the trajectory of the vehicle started to behave strange (oscillations appeared
, steering was very sudden)

---

## Challenges Faced

The biggest chalange faced was understanding the math behind this project - from the geometry behind computing headings to the formulas behind each controller. To be more specific, it was hard
for me to understand at first how each controller parameter affects the trajectory of the vehicle. 
Additionally, I found it hard at first working with Raylib, as it was my first time using this library. 
Nevertheless, in terms of the simulation, I find it difficult to tune the controllers in order to make the vehicle follow short paths when it has high speed.

# Simulation Results

### 1. Standard Models

Standard Bicycle Model 
![Standard Bicycle Model](docs/1.png)

Standard Model with Lateral Drift
![Standard Model with Lateral Drift](docs/2.png)

### 2. Heading Control

Straight Heading
![Straight Heading](docs/3.png)

Straight Heading with Drift & Noise
![Straight Heading with Drift & Noise](docs/4.png)

### 3. Straight Path Following

Straight Path – P Controller
![Straight Path – P Controller](docs/5.png)

Straight Path – PID Controller
![Straight Path – PID Controller](docs/6.png)

Straight Path – Stanley Controller
![Straight Path – Stanley Controller](docs/7.png)

### 4. Circular Path Following

Circular Path – P Controller
![Circular Path – P Controller](docs/8.png)

Circular Path – PID Controller
![Circular Path – PID Controller](docs/9.png)

Circular Path – Stanley Controller
![Circular Path – Stanley Controller](docs/10.png)

### 5. Sine Path Following

Sine Path – P Controller
![Sine Path – P Controller](docs/11.png)

Sine Path – PID Controller
![Sine Path – PID Controller](docs/12.png)

Sine Path – Stanley Controller
![Sine Path – Stanley Controller](docs/13.png)

### 6. Chicane Path Following

Chicane Path – PID Controller
![Chicane Path – PID Controller](docs/14.png)

Chicane Path – Stanley Controller
![Chicane Path – Stanley Controller](docs/15.png)

---

## Author
Dragos Baiu  
