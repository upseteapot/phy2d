#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>


typedef struct
{
    float x;
    float y;
} Phy2D_Vec2f;


// Shapeless rigid body.
typedef struct
{
    float restitution;
    float mass;
    float moment_of_inertia;
    Phy2D_Vec2f _last_position;
    Phy2D_Vec2f position;
    Phy2D_Vec2f velocity;
    float _last_angle;
    float angle;
    float angular_velocity;
} Phy2D_Body;

void Phy2D_SetPosition(Phy2D_Body *body, float pos_x, float pos_y);
void Phy2D_SetAngle(Phy2D_Body *body, float angle);
void Phy2D_SetVelocity(Phy2D_Body *body, float vel_x, float vel_y);
void Phy2D_SetAngularVelocity(Phy2D_Body *body, float angular_velocity);
void Phy2D_ApplyForce(Phy2D_Body *body, float for_x, float for_y, float point_x, float point_y, float dt);
void Phy2D_UpdateBody(Phy2D_Body *body, float dt);


// Disk shaped rigid body.
typedef struct
{
    Phy2D_Body body;
    float radius;
} Phy2D_DiskBody;

void Phy2D_CreateDisk(Phy2D_DiskBody *disk, float radius, float mass, float restitution);
void Phy2D_CreateSphere(Phy2D_DiskBody *disk, float radius, float mass, float restitution);


// State of the system.
typedef struct
{   
    size_t disk_array_capacity;
    size_t disk_array_size;
    Phy2D_DiskBody *disk_array;

    float static_friction;
    float dynamic_friction;
} Phy2D_State;

void Phy2D_CreateState(Phy2D_State *state, size_t disk_array_capacity);
void Phy2D_SetFriction(Phy2D_State *state, float static_friction, float dynamic_friction);
void Phy2D_AddDiskBody(Phy2D_State *state, float radius, float mass, float restitution);
void Phy2D_UpdateState(Phy2D_State *state, float dt);
void Phy2D_FreeState(Phy2D_State *state);


// Collision solver.
void Phy2D_SolveCollisions(Phy2D_State *state, float dt);

