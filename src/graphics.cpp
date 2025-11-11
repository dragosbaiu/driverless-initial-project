#include "graphics.hpp"
#include "vehicle.hpp"
#include "path.hpp"
#include <raylib.h>

using namespace std;

// Toggle fullscreen mode for the application window
void ToggleFullscreenWindow(){
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

// Run the graphical application to visualize the vehicle trajectory
void runApp(Vehicle& vehicle, double roundedMax, string windowTitle) {
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(800, 600, windowTitle.c_str());
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

        // Axis lines
        DrawLine(marginX - 10, centerY, screenWidth - marginX + 10, centerY, RAYWHITE);
        DrawLine(centerX, marginY -10, centerX, screenHeight - marginY + 10, RAYWHITE);
        
        // Axis labels
        DrawText("X", screenWidth-marginX+40, centerY-8, 16, RAYWHITE);
        DrawText("Y", centerX-5, marginY-40, 16, RAYWHITE);

        int i=1;
        // Draw X axis ticks and labels
        while (i*marginY <= centerX-marginX){
            DrawLine(centerX + (i*marginY), centerY - 5, centerX + (i*marginY), centerY + 5, RAYWHITE);
            DrawLine(centerX - (i*marginY), centerY - 5, centerX - (i*marginY), centerY + 5, RAYWHITE);

            DrawText(TextFormat("%.2f", i*roundedMax/4.00), centerX + (i*marginY) - 10, centerY + 8, 5, RAYWHITE);
            DrawText(TextFormat("-%.2f", i*roundedMax/4.00), centerX - (i*marginY) - 10, centerY + 8, 5, RAYWHITE);

            i++;
        }
        // Draw Y axis ticks and labels
        for (int i = 1; i < 5; i++){
            DrawLine(centerX - 5, centerY + (i*marginY), centerX + 5, centerY + (i*marginY), RAYWHITE);
            DrawLine(centerX - 5, centerY - (i*marginY), centerX + 5, centerY - (i*marginY), RAYWHITE);
            DrawText(TextFormat("%.2f", i*roundedMax/4.00), centerX + 7, centerY - (i*marginY) - 5, 5, RAYWHITE);
            DrawText(TextFormat("-%.2f", i*roundedMax/4.00), centerX  + 7, centerY + (i*marginY) - 5, 5, RAYWHITE);

        }
        
        double scale = (centerY - marginY) / roundedMax;
        // Draw vehicle trajectory
        for (int i = 0; i < vehicle.x.size(); i++){
            DrawCircle(centerX + vehicle.x[i] * scale, centerY - vehicle.y[i]* scale , 1, YELLOW);
        }

        EndDrawing();
    }

    CloseWindow();
}

void runApp(Path& path, double roundedMax, string windowTitle) {
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(800, 600, windowTitle.c_str());
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

        // Axis lines
        DrawLine(marginX - 10, centerY, screenWidth - marginX + 10, centerY, RAYWHITE);
        DrawLine(centerX, marginY -10, centerX, screenHeight - marginY + 10, RAYWHITE);
        
        // Axis labels
        DrawText("X", screenWidth-marginX+40, centerY-8, 16, RAYWHITE);
        DrawText("Y", centerX-5, marginY-40, 16, RAYWHITE);

        int i=1;
        // Draw X axis ticks and labels
        while (i*marginY <= centerX-marginX){
            DrawLine(centerX + (i*marginY), centerY - 5, centerX + (i*marginY), centerY + 5, RAYWHITE);
            DrawLine(centerX - (i*marginY), centerY - 5, centerX - (i*marginY), centerY + 5, RAYWHITE);

            DrawText(TextFormat("%.2f", i*roundedMax/4.00), centerX + (i*marginY) - 10, centerY + 8, 5, RAYWHITE);
            DrawText(TextFormat("-%.2f", i*roundedMax/4.00), centerX - (i*marginY) - 10, centerY + 8, 5, RAYWHITE);

            i++;
        }
        // Draw Y axis ticks and labels
        for (int i = 1; i < 5; i++){
            DrawLine(centerX - 5, centerY + (i*marginY), centerX + 5, centerY + (i*marginY), RAYWHITE);
            DrawLine(centerX - 5, centerY - (i*marginY), centerX + 5, centerY - (i*marginY), RAYWHITE);
            DrawText(TextFormat("%.2f", i*roundedMax/4.00), centerX + 7, centerY - (i*marginY) - 5, 5, RAYWHITE);
            DrawText(TextFormat("-%.2f", i*roundedMax/4.00), centerX  + 7, centerY + (i*marginY) - 5, 5, RAYWHITE);

        }
        
        double scale = (centerY - marginY) / roundedMax;
        // Draw vehicle trajectory
        for (int i = 0; i < path.x.size(); i++){
            DrawCircle(centerX + path.x[i] * scale, centerY - path.y[i]* scale , 2, BLUE);
        }

        EndDrawing();
    }

    CloseWindow();
}