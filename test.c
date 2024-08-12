#include <stdio.h>
#include <raylib.h>
#include <time.h>

#include "phy2d.h"


int main(void)
{
    srand(time(0));

    InitWindow(0, 0, "Collision");
    ToggleFullscreen();

    int width = GetRenderWidth();
    int height = GetRenderHeight();
    printf("%d %d\n", width, height);

    float sim_dt = 0.0005f;
    float sim_time = 0.0f; 
    
    // Create circles.
    float radius = 10.0f;
    size_t balls = 100;
    
    Phy2D_State state;
    Phy2D_CreateState(&state, balls, 1);
        
    Color colors[balls];
    
    for (size_t i=0; i < balls; i++)
    {
        colors[i] = RED;
        Phy2D_AddDiskBody(&state, radius, 1.0f, 1.0f);    
        Phy2D_SetPosition(&Phy2D_GetDiskBody(&state, i)->body, ((float)rand() / RAND_MAX) * width, ((float)rand() / RAND_MAX) * height);
    }

    Phy2D_AddPolyBody(&state, 4, 1.0f, 1.0f);
    
    Phy2D_PolyBody *b = Phy2D_GetPolyBody(&state, 0);
    Phy2D_AddPolyPoint(b, -10.0f, 5.0f);
    Phy2D_AddPolyPoint(b,  10.0f, 5.0f);
    Phy2D_AddPolyPoint(b,  10.0f, -5.0f);
    Phy2D_AddPolyPoint(b, -10.0f, -5.0f);
    
    Phy2D_SetPosition(&b->body, 200, 200);
    Phy2D_SetAngularVelocity(&b->body, 1.0f);

    colors[0] = YELLOW;

    // Start main loop.
    float elapsed_time;
    while (!WindowShouldClose())
    {
        elapsed_time = GetFrameTime();
    
        sim_time = 0.0f;
        while (sim_time < elapsed_time)
        {
            // Update circles velocities.
            if (IsMouseButtonDown(MOUSE_LEFT_BUTTON))
            {
                Vector2 position = GetMousePosition(); 

                Vector2 dir;
                dir.x = position.x - state.disk_array[0].body.position.x;
                dir.y = position.y - state.disk_array[0].body.position.y;
                float dir_mag = sqrt(dir.x * dir.x + dir.y * dir.y);
            
                Phy2D_ApplyForce(&state.disk_array[0].body, 200.0f * dir.x / dir_mag, 200.0f * dir.y / dir_mag, 0.0f, 0.0f, sim_dt);
            }
            
            // Update circles position.
            Phy2D_UpdateState(&state, sim_dt);
            Phy2D_SolveCollisions(&state, sim_dt);
            
            sim_time += sim_dt;
        }

        BeginDrawing();
        ClearBackground(BLACK);

        // Draw circles on screen.
        for (size_t i=0; i < balls; i++)
        {
            DrawCircle(state.disk_array[i].body.position.x, state.disk_array[i].body.position.y, radius, colors[i]);
        
            Vector2 line_s;
            line_s.x = state.disk_array[i].body.position.x;
            line_s.y = state.disk_array[i].body.position.y;

            Vector2 line_e;
            line_e.x = state.disk_array[i].body.position.x + cos(state.disk_array[i].body.angle) * radius;
            line_e.y = state.disk_array[i].body.position.y - sin(state.disk_array[i].body.angle) * radius;

            DrawLineV(line_s, line_e, BLUE);
        }

        Phy2D_PolyBody *pb = Phy2D_GetPolyBody(&state, 0);
        Phy2D_GetTransformedPoints(pb);
        Vector2 points[6];
        
        points[0].x = pb->body.position.x;
        points[0].y = pb->body.position.y;

        for (size_t i=1; i < 5; i++)
        {
            points[i].x = pb->transf_points[i-1].x;
            points[i].y = pb->transf_points[i-1].y;
        }
        
        points[5].x = pb->transf_points[0].x;
        points[5].y = pb->transf_points[0].y;

        DrawTriangleFan(points, 6, BLUE);

        EndDrawing();
    }

    Phy2D_FreeState(&state);

    return 0;
}

