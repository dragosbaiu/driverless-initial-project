#include "graphics.hpp"
#include "vehicle.hpp"
#include <raylib.h>

using namespace std;

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

void runApp(const Vehicle& vehicle, double roundedMax) {
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

        for (int i = 0; i < vehicle.x.size(); i++){
            DrawCircle(centerX + vehicle.x[i] * scale, centerY - vehicle.y[i]* scale , 1, YELLOW);
        }

        EndDrawing();
    }

    CloseWindow();
}

