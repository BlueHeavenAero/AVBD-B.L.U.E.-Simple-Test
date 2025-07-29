/*
* Copyright (c) 2025 Chris Giles
*
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Chris Giles makes no representations about the suitability
* of this software for any purpose.
* It is provided "as is" without express or implied warranty.
*/

#include "solver.h"

// Constructor for boxes
Rigid::Rigid(Solver* solver, float2 size, float density, float friction, float3 position, float3 velocity)
    : solver(solver), forces(0), next(0), position(position), velocity(velocity), prevVelocity(velocity), size(size), friction(friction), shapeType(SHAPE_BOX)
{
    // Add to linked list
    next = solver->bodies;
    solver->bodies = this;

    // Compute mass properties and bounding radius for boxes
    mass = size.x * size.y * density;
    moment = mass * dot(size, size) / 12.0f;
    radius = length(size * 0.5f);
}

// Constructor for circles
Rigid::Rigid(Solver* solver, float circleRadius, float density, float friction, float3 position, float3 velocity)
    : solver(solver), forces(0), next(0), position(position), velocity(velocity), prevVelocity(velocity), friction(friction), shapeType(SHAPE_CIRCLE)
{
    // Add to linked list
    next = solver->bodies;
    solver->bodies = this;

    // Store circle radius in size.x for consistency
    size = float2{ circleRadius, circleRadius };
    
    // Compute mass properties for circles
    // Mass = π * r² * density
    // Moment of inertia = 0.5 * mass * r²
    float r = circleRadius;
    mass = 3.14159f * r * r * density;
    moment = 0.5f * mass * r * r;
    radius = r; // Bounding radius is just the circle radius
}

Rigid::~Rigid()
{
    // Remove from linked list
    Rigid** p = &solver->bodies;
    while (*p != this)
        p = &(*p)->next;
    *p = next;
}

bool Rigid::constrainedTo(Rigid* other) const
{
    // Check if this body is constrained to the other body
    for (Force* f = forces; f != 0; f = f->next)
        if ((f->bodyA == this && f->bodyB == other) || (f->bodyA == other && f->bodyB == this))
            return true;
    return false;
}

void Rigid::draw()
{
    if (shapeType == SHAPE_BOX)
    {
        // Draw box
        float2x2 R = rotation(position.z);
        float2 v0 = R * float2{ -size.x * 0.5f, -size.y * 0.5f } + position.xy();
        float2 v1 = R * float2{ size.x * 0.5f, -size.y * 0.5f } + position.xy();
        float2 v2 = R * float2{ size.x * 0.5f, size.y * 0.5f } + position.xy();
        float2 v3 = R * float2{ -size.x * 0.5f, size.y * 0.5f } + position.xy();

        glColor3f(0.6f, 0.6f, 0.6f);
        glBegin(GL_QUADS);
        glVertex2f(v0.x, v0.y);
        glVertex2f(v1.x, v1.y);
        glVertex2f(v2.x, v2.y);
        glVertex2f(v3.x, v3.y);
        glEnd();

        glColor3f(0, 0, 0);
        glBegin(GL_LINE_LOOP);
        glVertex2f(v0.x, v0.y);
        glVertex2f(v1.x, v1.y);
        glVertex2f(v2.x, v2.y);
        glVertex2f(v3.x, v3.y);
        glEnd();
    }
    else if (shapeType == SHAPE_CIRCLE)
    {
        // Draw circle
        float2 center = position.xy();
        float r = size.x; // Circle radius stored in size.x
        
        // Fill
        glColor3f(0.6f, 0.6f, 0.6f);
        glBegin(GL_TRIANGLE_FAN);
        glVertex2f(center.x, center.y); // Center point
        for (int i = 0; i <= 32; i++)
        {
            float angle = i * 2.0f * 3.14159f / 32.0f;
            glVertex2f(center.x + cosf(angle) * r, center.y + sinf(angle) * r);
        }
        glEnd();

        // Outline
        glColor3f(0, 0, 0);
        glBegin(GL_LINE_LOOP);
        for (int i = 0; i < 32; i++)
        {
            float angle = i * 2.0f * 3.14159f / 32.0f;
            glVertex2f(center.x + cosf(angle) * r, center.y + sinf(angle) * r);
        }
        glEnd();

        // Draw radius line to show rotation
        float angle = position.z;
        glColor3f(0, 0, 0);
        glBegin(GL_LINES);
        glVertex2f(center.x, center.y);
        glVertex2f(center.x + cosf(angle) * r, center.y + sinf(angle) * r);
        glEnd();
    }
}
