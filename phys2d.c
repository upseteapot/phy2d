#include "phys2d.h"


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


// States of the system.
void Phy2D_CreateState(Phy2D_State *state, size_t disk_array_capacity)
{
    state->disk_array_capacity = disk_array_capacity;
    state->disk_array_size = 0.0f;
    state->disk_array = (Phy2D_DiskBody *)malloc(sizeof(Phy2D_DiskBody) * disk_array_capacity);

    state->static_friction = 0.0f;
    state->dynamic_friction = 0.0f;
}

void Phy2D_SetFriction(Phy2D_State *state, float static_friction, float dynamic_friction)
{
    state->static_friction = static_friction;
    state->dynamic_friction = dynamic_friction;
}

void Phy2D_AddDiskBody(Phy2D_State *state, float radius, float mass, float restitution)
{
    if (state->disk_array_size < state->disk_array_capacity)
    {
        Phy2D_CreateDisk(&state->disk_array[state->disk_array_size], radius, mass, restitution);
        state->disk_array_size++;
    } else
    {
        printf("[PHY2D] Disk array max capacity reached. (SIZE=%ld)\n", state->disk_array_capacity);
    }
}

void Phy2D_UpdateState(Phy2D_State *state, float dt)
{
    // Update Disk rigid bodies
    for (size_t i=0; i < state->disk_array_size; i++)
        Phy2D_UpdateBody(&state->disk_array[i].body, dt);
}

void Phy2D_FreeState(Phy2D_State *state)
{
    free(state->disk_array);
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

//static inline float GetDiskCollisionInstant(Phy2D_DiskBody *disk_a, Phy2D_DiskBody *disk_b)
//{
//    float a = pow(disk_a->body.velocity.x - disk_b->body.velocity.x, 2) + pow(disk_a->body.velocity.y - disk_b->body.velocity.y, 2);
//    float b = 2 * (disk_a->body.position.x - disk_b->body.position.x) * (disk_a->body.velocity.x - disk_b->body.velocity.x) +
//              2 * (disk_a->body.position.y - disk_b->body.position.y) * (disk_a->body.velocity.y - disk_b->body.velocity.y);
//    float c = pow(disk_a->body.position.x - disk_b->body.position.x, 2) + 
//              pow(disk_a->body.position.y - disk_b->body.position.y, 2) -
//              pow(disk_a->radius + disk_b->radius, 2);
//    
//    float t1 = (-b + sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
//    float t2 = (-b - sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
//    return fmin(t1, t2);
//}

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
static void HandleCollision(Phy2D_Body *a, Phy2D_Body *b, Collision *collision, float static_friction, float dynamic_friction)
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
                        &collision, state->static_friction, 
                        state->dynamic_friction
                        );
            }
        }
    }
}

