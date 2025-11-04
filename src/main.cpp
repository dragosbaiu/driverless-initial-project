#include "vehicle.hpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <raylib.h>

using namespace std;

void ToggleFullscreen() {
    static bool isFullscreen = false;
    isFullscreen = !isFullscreen;
    if (isFullscreen) {
        int monitor = GetCurrentMonitor();
        SetWindowSize(GetMonitorWidth(monitor), GetMonitorHeight(monitor));
        ToggleFullscreen();
    } else {
        ToggleFullscreen();
        SetWindowSize(800, 600);
    }
}

int main() {
    
    double max;

    ifstream file ("data/input.txt");
    if (!file){
        cerr << "Unable to open file input.txt";
        return 1;
    }

    Vehicle bycicle;
    file >> bycicle.x[0] >> bycicle.y[0] >> bycicle.theta[0] >> bycicle.velocity[0];
    file >> bycicle.acceleration >> bycicle.delta >> bycicle.dt >> bycicle.L;

    /*cout << "Initial Position: x = " << bycicle.x << ", y = " << bycicle.y << ", theta = " 
        << bycicle.theta  << ", Velocity: " << bycicle.velocity << ", Delta: " << 
        delta << ", Time Step: " << dt << ", Wheelbase: " << L << endl;
    */

    for (int i = 1; i < 100; i++){
        bycicle.updatePosition();
        if (abs(bycicle.x[i]) > max) {
            max = abs(bycicle.x[i]);
        }
        if (abs(bycicle.y[i]) > max) {
            max = abs(bycicle.y[i]);
        }
        //cout << "Position " << i+1 << ": x = " << bycicle.x << ", y = " << bycicle.y << ", theta = " << bycicle.theta << endl;
    }

    double roundedMax = ceil(max);

    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(800, 600, "Vehicle Trajectory");
    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        
        BeginDrawing();
        ClearBackground(BLACK);
        int screenWidth = GetScreenWidth();
        int screenHeight = GetScreenHeight();

        int marginX = screenWidth/10;
        int marginY = screenHeight/10;
        int centerX = screenWidth/2;
        int centerY = screenHeight/2;

        DrawLine(marginX - 10, centerY, screenWidth - marginX + 10, centerY, RAYWHITE);
        DrawLine(centerX, marginY -10, centerX, screenHeight - marginY + 10, RAYWHITE);
        
        DrawText("X", screenWidth-marginX+40, centerY-8, 16, RAYWHITE);
        DrawText("Y", centerX-5, marginY-40, 16, RAYWHITE);

        int i=1;
        while (i*marginY <= centerX-marginX){
            DrawLine(centerX + (i*marginY), centerY - 5, centerX + (i*marginY), centerY + 5, RAYWHITE);
            DrawLine(centerX - (i*marginY), centerY - 5, centerX - (i*marginY), centerY + 5, RAYWHITE);

            DrawText(TextFormat("%.2f", i*roundedMax/4.00), centerX + (i*marginY) - 10, centerY + 8, 5, RAYWHITE);
            DrawText(TextFormat("-%.2f", i*roundedMax/4.00), centerX - (i*marginY) - 10, centerY + 8, 5, RAYWHITE);

            i++;
        }
        for (int i = 1; i < 5; i++){
            DrawLine(centerX - 5, centerY + (i*marginY), centerX + 5, centerY + (i*marginY), RAYWHITE);
            DrawLine(centerX - 5, centerY - (i*marginY), centerX + 5, centerY - (i*marginY), RAYWHITE);
            DrawText(TextFormat("%.2f", i*roundedMax/4.00), centerX + 7, centerY - (i*marginY) - 5, 5, RAYWHITE);
            DrawText(TextFormat("-%.2f", i*roundedMax/4.00), centerX  + 7, centerY + (i*marginY) - 5, 5, RAYWHITE);

        }
        
        double scale = (centerY - marginY) / roundedMax;

        for (int i = 0; i < bycicle.x.size(); i++){
            DrawCircle(centerX + bycicle.x[i] * scale, centerY - bycicle.y[i]* scale , 1, YELLOW);
        }

        EndDrawing();
    }

    CloseWindow();

    file.close();

    
    return 0;
}


// Make array to save all the positions of x and y, in the for to create them all
// also store max coordinate. Change the hpp and cpp file to make sure theta, x, 
//and y are arrays of values

