#include "phy2d.h"


// Helper functions.
static float DotProduct(Phy2D_Vec2f a, Phy2D_Vec2f b)
{
    return a.x * b.x + a.y * b.y;
}

static float PerpDotProduct(Phy2D_Vec2f a, Phy2D_Vec2f b)
{
    return a.x * b.y - a.y * b.x;
}


// Shapeless rigid body.
static inline void BackTrack(Phy2D_Body *body)
{
    body->position = body->_last_position;
    body->angle = body->_last_angle;
}

void Phy2D_CreateBody(Phy2D_Body *body, float mass, float restitution)
{
    body->restitution = restitution;
    body->mass = mass;
    body->moment_of_inertia = 1.0f; // Default value
    body->_last_position = (Phy2D_Vec2f){ 0.0f, 0.0f };
    body->position = (Phy2D_Vec2f){ 0.0f, 0.0f };
    body->velocity = (Phy2D_Vec2f){ 0.0f, 0.0f };
    body->_last_angle = 0.0f;
    body->angle = 0.0f;
    body->angular_velocity = 0.0f;
}

void Phy2D_SetPosition(Phy2D_Body *body, float pos_x, float pos_y)
{
    body->_last_position = body->position;
    body->position.x = pos_x;
    body->position.y = pos_y;
}

void Phy2D_SetAngle(Phy2D_Body *body, float angle)
{
    body->_last_angle = body->angle;
    body->angle = angle;
}

void Phy2D_SetVelocity(Phy2D_Body *body, float vel_x, float vel_y)
{
    body->velocity.x = vel_x;
    body->velocity.y = vel_y;
}

void Phy2D_SetAngularVelocity(Phy2D_Body *body, float angular_velocity)
{
    body->angular_velocity = angular_velocity;
}

void Phy2D_ApplyForce(Phy2D_Body *body, float for_x, float for_y, float point_x, float point_y, float dt)
{
    body->angular_velocity += PerpDotProduct((Phy2D_Vec2f){ point_x, point_y }, (Phy2D_Vec2f){ for_x, for_y }) * dt / body->moment_of_inertia;
    body->velocity.x += for_x * dt / body->mass;
    body->velocity.y += for_y * dt / body->mass;
}

void Phy2D_UpdateBody(Phy2D_Body *body, float dt)
{
    body->_last_position = body->position;
    body->position.x += body->velocity.x * dt;
    body->position.y += body->velocity.y * dt;
    
    body->_last_angle = body->angle;
    body->angle += body->angular_velocity * dt;
}


// Disk shaped rigid body.
void Phy2D_CreateDisk(Phy2D_DiskBody *disk, float radius, float mass, float restitution)
{
    Phy2D_CreateBody(&disk->body, mass, restitution);
    disk->radius = radius;
    disk->body.moment_of_inertia = 0.5f * mass * pow(disk->radius, 2); 
}

void Phy2D_CreateSphere(Phy2D_DiskBody *disk, float radius, float mass, float restitution)
{
    Phy2D_CreateBody(&disk->body, mass, restitution);
    disk->radius = radius;
    disk->body.moment_of_inertia = 2.0f / 5.0f * mass * pow(disk->radius, 2); 
}


// Polygon rigid body.
void Phy2D_CreatePoly(Phy2D_PolyBody *poly, size_t size, float mass, float restitution)
{
    Phy2D_CreateBody(&poly->body, mass, restitution);
    poly->size = size;
    poly->_index = 0;
    poly->points = (Phy2D_Vec2f *)malloc(sizeof(Phy2D_Vec2f) * size);
    poly->transf_points = (Phy2D_Vec2f *)malloc(sizeof(Phy2D_Vec2f) * size);
}

void Phy2D_AddPolyPoint(Phy2D_PolyBody *poly, float x, float y)
{
    if (poly->_index >= poly->size)
    {
        printf("[PHY2D] Polygon's points array max capacity reached. (SIZE=%ld)\n", poly->size);
        return;
    }

    poly->points[poly->_index] = (Phy2D_Vec2f){ x, y };

    if (++poly->_index == poly->size)
    {
        Phy2D_Vec2f center_of_mass = (Phy2D_Vec2f){ 0.0f, 0.0f };

        // Calculate center of mass.
        for (size_t i=0; i < poly->size; i++)
        {
            center_of_mass.x += poly->points[i].x / (float)poly->size;
            center_of_mass.y += poly->points[i].y / (float)poly->size;
        }

        // Move all points.
        for (size_t i=0; i < poly->size; i++)
        {
            poly->points[i].x -= center_of_mass.x;
            poly->points[i].y -= center_of_mass.y;
        }

        // Calculate MOI.
        float sma = 0; // Second moment of area. 
        float j_x;
        float j_y;
        for (size_t i=0; i < poly->size-1; i++)
        {
            j_x = (poly->points[i].x * poly->points[i+1].y - poly->points[i+1].x * poly->points[i].y)
                * (pow(poly->points[i].y, 2) + poly->points[i].y * poly->points[i+1].y + pow(poly->points[i+1].y, 2));

            j_y = (poly->points[i].x * poly->points[i+1].y - poly->points[i+1].x * poly->points[i].y)
                * (pow(poly->points[i].x, 2) + poly->points[i].x * poly->points[i+1].x + pow(poly->points[i+1].x, 2));

            sma += (1.0f / 2.0f) * (j_x + j_y);
        }

        poly->body.moment_of_inertia = sma * poly->body.mass;
    }
}

void Phy2D_GetTransformedPoints(Phy2D_PolyBody *poly)
{
    for (size_t i=0; i < poly->size; i++)
    {
        poly->transf_points[i].x = poly->body.position.x + cos(poly->body.angle) * poly->points[i].x + sin(poly->body.angle) * poly->points[i].y; 
        poly->transf_points[i].y = poly->body.position.y + -sin(poly->body.angle) * poly->points[i].x + cos(poly->body.angle) * poly->points[i].y; 
    }
}

void Phy2D_FreePoly(Phy2D_PolyBody *poly)
{
    free(poly->points);
    free(poly->transf_points);
}


// States of the system.
void Phy2D_CreateState(Phy2D_State *state, size_t disk_array_capacity, size_t poly_array_capacity)
{
    if (disk_array_capacity > 0)
    {
        state->disk_array_capacity = disk_array_capacity;
        state->disk_array_size = 0;
        state->disk_array = (Phy2D_DiskBody *)malloc(sizeof(Phy2D_DiskBody) * disk_array_capacity);
    }

    if (poly_array_capacity > 0) 
    {   
        state->poly_array_capacity = poly_array_capacity;
        state->poly_array_size = 0;
        state->poly_array = (Phy2D_PolyBody *)malloc(sizeof(Phy2D_PolyBody) * poly_array_capacity);
    }
}

void Phy2D_AddDiskBody(Phy2D_State *state, float radius, float mass, float restitution)
{
    if (state->disk_array_size >= state->disk_array_capacity)
    {
        printf("[PHY2D] Disk array max capacity reached. (SIZE=%ld)\n", state->disk_array_capacity);
        return; 
    }

    Phy2D_CreateDisk(&state->disk_array[state->disk_array_size], radius, mass, restitution);
    state->disk_array_size++;
}

void Phy2D_AddPolyBody(Phy2D_State *state, size_t size, float mass, float restitution)
{
    if (state->poly_array_size >= state->poly_array_capacity)
    {
        printf("[PHY2D] Polygon array max capacity reached. (SIZE=%ld)\n", state->poly_array_capacity);
        return; 
    }

    Phy2D_CreatePoly(&state->poly_array[state->poly_array_size], size, mass, restitution);
    state->poly_array_size++;
}

Phy2D_DiskBody* Phy2D_GetDiskBody(Phy2D_State *state, size_t i)
{
    if (i >= state->disk_array_size)
    {
        printf("[PHY2D] Disk index out of range. (SIZE=%ld)\n", state->disk_array_size);
        return NULL;
    }

    return &state->disk_array[i];
}

Phy2D_PolyBody* Phy2D_GetPolyBody(Phy2D_State *state, size_t i)
{
    if (i >= state->poly_array_size)
    {
        printf("[PHY2D] Polygon index out of range. (SIZE=%ld)\n", state->poly_array_size);
        return NULL;
    }

    return &state->poly_array[i];
}

void Phy2D_UpdateState(Phy2D_State *state, float dt)
{
    // Update Disk rigid bodies
    for (size_t i=0; i < state->disk_array_size; i++)
        Phy2D_UpdateBody(&state->disk_array[i].body, dt);
    
    // Update Poly rigid bodies
    for (size_t i=0; i < state->poly_array_size; i++)
        Phy2D_UpdateBody(&state->poly_array[i].body, dt);
}

void Phy2D_FreeState(Phy2D_State *state)
{
    if (state->disk_array_capacity > 0)
        free(state->disk_array);

    if (state->poly_array_capacity > 0)
    {
        for (size_t i=0; i < state->poly_array_size; i++)
            Phy2D_FreePoly(&state->poly_array[i]);

        free(state->poly_array);
    }
}


// Collision structure.
typedef struct
{
    float depth;
    Phy2D_Vec2f normal;
    Phy2D_Vec2f contact_a;
    Phy2D_Vec2f contact_b;
} Collision;


// Disk-Disk collision.
static inline bool IsColliding_DISDIS(Phy2D_DiskBody *disk_a, Phy2D_DiskBody *disk_b)
{
    Phy2D_Vec2f dist;
    dist.x = disk_a->body.position.x - disk_b->body.position.x;
    dist.y = disk_a->body.position.y - disk_b->body.position.y;
    return DotProduct(dist, dist) < pow(disk_a->radius + disk_b->radius, 2);
}

static void GetCollisionInformation_DISDIS(Collision *collision, Phy2D_DiskBody *disk_a, Phy2D_DiskBody *disk_b)
{
    Phy2D_Vec2f dist;
    dist.x = disk_a->body.position.x - disk_b->body.position.x;
    dist.y = disk_a->body.position.y - disk_b->body.position.y;

    float max_dist = disk_a->radius + disk_b->radius;
    float dist_mag = sqrt(DotProduct(dist, dist));

    collision->depth = (max_dist - dist_mag) / 2.0f;
    collision->normal.x = dist.x / dist_mag;
    collision->normal.y = dist.y / dist_mag;

    collision->contact_a.x = -collision->normal.x * disk_a->radius;
    collision->contact_a.y = -collision->normal.y * disk_a->radius;
    collision->contact_b.x = collision->normal.x * disk_b->radius; 
    collision->contact_b.y = collision->normal.y * disk_b->radius; 
}


// Collision solver.
static void HandleCollision(Phy2D_Body *a, Phy2D_Body *b, Collision *collision)
{
    float e = fmin(a->restitution, b->restitution);

    Phy2D_Vec2f rvel;
    rvel.x = a->velocity.x - b->velocity.x; 
    rvel.y = a->velocity.y - b->velocity.y; 

    a->position.x += collision->normal.x * collision->depth;
    a->position.y += collision->normal.y * collision->depth;
    b->position.x -= collision->normal.x * collision->depth;
    b->position.y -= collision->normal.y * collision->depth;

    float velocity_normal = DotProduct(collision->normal, rvel);

    if (velocity_normal == 0)
        return;

    // Normal component of interaction (j normal)
    float j_normal = -(1.0f + e) * velocity_normal
        / (1.0f / a->mass + 1.0f / b->mass
                + pow(PerpDotProduct(collision->contact_a, collision->normal), 2) / a->moment_of_inertia
                + pow(PerpDotProduct(collision->contact_b, collision->normal), 2) / b->moment_of_inertia);

    Phy2D_Vec2f impulse;
    impulse.x = collision->normal.x * j_normal;
    impulse.y = collision->normal.y * j_normal;

    a->velocity.x += impulse.x / a->mass;
    a->velocity.y += impulse.y / a->mass;
    b->velocity.x -= impulse.x / b->mass;
    b->velocity.y -= impulse.y / b->mass;

    a->angular_velocity += PerpDotProduct(collision->contact_a, impulse) / a->moment_of_inertia;
    b->angular_velocity -= PerpDotProduct(collision->contact_b, impulse) / b->moment_of_inertia;
}

void Phy2D_SolveCollisions(Phy2D_State *state, float dt)
{
    // Test disk bodies against other disk bodies.
    for (size_t i=0; i < state->disk_array_size; i++)
    {
        for (size_t j=i+1; j < state->disk_array_size; j++)
        {
            if (IsColliding_DISDIS(&state->disk_array[i], &state->disk_array[j]))
            {
                Collision collision;
                GetCollisionInformation_DISDIS(&collision, &state->disk_array[i], &state->disk_array[j]);

                HandleCollision(
                        &state->disk_array[i].body, 
                        &state->disk_array[j].body, 
                        &collision 
                        );
            }
        }
    }
}

