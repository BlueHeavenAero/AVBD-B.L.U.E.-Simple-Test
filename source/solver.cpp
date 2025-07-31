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
#include <cstdio>
#include <cstdlib>  // For rand(), srand()
#include <ctime>    // For time()
#include <cmath>    // For logf(), cosf(), sinf(), sqrtf()

Solver::Solver()
    : bodies(0), forces(0)
{
    defaultParams();
}

Solver::~Solver()
{
    clear();
}

Rigid* Solver::pick(float2 at, float2& local)
{
    // Find which body is at the given point
    for (Rigid* body = bodies; body != 0; body = body->next)
    {
        float2x2 Rt = rotation(-body->position.z);
        local = Rt * (at - body->position.xy());
        if (local.x >= -body->size.x * 0.5f && local.x <= body->size.x * 0.5f &&
            local.y >= -body->size.y * 0.5f && local.y <= body->size.y * 0.5f)
            return body;
    }
    return 0;
}

void Solver::clear()
{
    while (forces)
        delete forces;

    while (bodies)
        delete bodies;
}

void Solver::defaultParams()
{
    dt = 1.0f / 60.0f;
    gravity = -10.0f;
    iterations = 10;

    // Note: in the paper, beta is suggested to be [1, 1000]. Technically, the best choice will
    // depend on the length, mass, and constraint function scales (ie units) of your simulation,
    // along with your strategy for incrementing the penalty parameters.
    // If the value is not in the right range, you may see slower convergance for complex scenes.
    beta = 100000.0f;

    // Alpha controls how much stabilization is applied. Higher values give slower and smoother
    // error correction, and lower values are more responsive and energetic. Tune this depending
    // on your desired constraint error response.
    alpha = 0.99f;

    // Gamma controls how much the penalty and lambda values are decayed each step during warmstarting.
    // This should always be < 1 so that the penalty values can decrease (unless you use a different
    // penalty parameter strategy which does not require decay).
    gamma = 0.99f;

    // Post stabilization applies an extra iteration to fix positional error.
    // This removes the need for the alpha parameter, which can make tuning a little easier.
    postStabilize = true;

    // Set simple hard-coded boundaries for testing
    // These work well with the default camera position (0, 5) and zoom 25
    boundaryLeft = -20.0f;
    boundaryRight = 20.0f;
    boundaryBottom = -5.0f;
    boundaryTop = 50.0f; // Increased by factor of 2 (was 25.0f)
    boundaryRestitution = 0.8f; // 80% bounce
    
    // Initialize circle dropping system
    circleDropEnabled = false;
    nextDropTime = 0.0f;
    currentTime = 0.0f;
    dropSystemInitialized = false;
    circlesDropped = 0;
    maxCirclesToDrop = 50;
}

void Solver::enableCircleDrop(bool enable)
{
    circleDropEnabled = enable;
    if (enable && !dropSystemInitialized) {
        // Initialize the dropping system
        srand((unsigned int)time(nullptr));
        currentTime = 0.0f;
        nextDropTime = 0.2f; // First drop after 0.2 seconds
        dropSystemInitialized = true;
        circlesDropped = 0;
    }
}

void Solver::resetCircleDrop()
{
    circlesDropped = 0;
    currentTime = 0.0f;
    nextDropTime = 0.2f;
    dropSystemInitialized = false;
}

void Solver::updateCircleDrop()
{
    if (!circleDropEnabled || circlesDropped >= maxCirclesToDrop) return;
    
    // Update time
    currentTime += dt;
    
    // Check if it's time to drop a new circle
    if (currentTime >= nextDropTime) {
        // Drop interval: average 5 per second = 0.2 seconds average
        // Use exponential distribution for realistic random intervals
        float averageInterval = 0.2f;
        float randomInterval = -averageInterval * logf((float)rand() / RAND_MAX);
        nextDropTime = currentTime + randomInterval;
        
        // Random horizontal position (normal distribution around center)
        // Box-Muller transform for normal distribution
        static bool hasSpare = false;
        static float spare;
        float randomX;
        
        if (hasSpare) {
            randomX = spare;
            hasSpare = false;
        } else {
            hasSpare = true;
            float u = (float)rand() / RAND_MAX;
            float v = (float)rand() / RAND_MAX;
            float mag = 3.0f * sqrtf(-2.0f * logf(u)); // 3.0f is the standard deviation
            randomX = mag * cosf(2.0f * 3.14159f * v);
            spare = mag * sinf(2.0f * 3.14159f * v);
        }
        
        // Clamp to 20% of boundary width
        float boundaryWidth = boundaryRight - boundaryLeft;
        float maxOffset = boundaryWidth * 0.2f; // 20% of boundary width
        randomX = clamp(randomX, -maxOffset, maxOffset);
        
        // Random initial rotation
        float randomRotation = ((float)rand() / RAND_MAX) * 2.0f * 3.14159f;
        
        // Drop height: from the boundary top
        float dropHeight = boundaryTop;
        
        // Create the circle with random position and rotation
        float circleRadius = 1.0f;
        new Rigid(this, circleRadius, 1.0f, 0.5f, 
                  { randomX, dropHeight, randomRotation });
        
        // Increment circle count
        circlesDropped++;
    }
}

void Solver::createBoundaryWalls()
{
    const float wallThickness = 1.0f;
    const float wallExtension = 5.0f; // Make walls slightly larger than needed
    
    // Create invisible static walls as rigid bodies
    // Left wall
    Rigid* leftWall = new Rigid(this, { wallThickness, boundaryTop - boundaryBottom + wallExtension * 2 }, 
                               0.0f, 0.5f, { boundaryLeft - wallThickness * 0.5f, (boundaryTop + boundaryBottom) * 0.5f, 0.0f });
    leftWall->invisible = true;
    
    // Right wall  
    Rigid* rightWall = new Rigid(this, { wallThickness, boundaryTop - boundaryBottom + wallExtension * 2 }, 
                                0.0f, 0.5f, { boundaryRight + wallThickness * 0.5f, (boundaryTop + boundaryBottom) * 0.5f, 0.0f });
    rightWall->invisible = true;
    
    // Bottom wall
    Rigid* bottomWall = new Rigid(this, { boundaryRight - boundaryLeft + wallExtension * 2, wallThickness }, 
                                 0.0f, 0.5f, { (boundaryRight + boundaryLeft) * 0.5f, boundaryBottom - wallThickness * 0.5f, 0.0f });
    bottomWall->invisible = true;
    
    // Top wall
    Rigid* topWall = new Rigid(this, { boundaryRight - boundaryLeft + wallExtension * 2, wallThickness }, 
                              0.0f, 0.5f, { (boundaryRight + boundaryLeft) * 0.5f, boundaryTop + wallThickness * 0.5f, 0.0f });
    topWall->invisible = true;
}

void Solver::step()
{
    // DEBUG: Count circle-circle contacts for this frame
    // int circleContactCount = 0;
    
    // Update circle dropping system
    updateCircleDrop();
    
    // Perform broadphase collision detection
    // This is a naive O(n^2) approach, but it is sufficient for small numbers of bodies in this sample.
    for (Rigid* bodyA = bodies; bodyA != 0; bodyA = bodyA->next)
    {
        for (Rigid* bodyB = bodyA->next; bodyB != 0; bodyB = bodyB->next)
        {
            float2 dp = bodyA->position.xy() - bodyB->position.xy();
            float r = bodyA->radius + bodyB->radius;
            if (dot(dp, dp) <= r * r && !bodyA->constrainedTo(bodyB))
            {
                // if (bodyA->shapeType == SHAPE_CIRCLE && bodyB->shapeType == SHAPE_CIRCLE) {
                //     circleContactCount++;
                // }
                new Manifold(this, bodyA, bodyB);
            }
        }
    }
    
    // if (circleContactCount > 2) {
    //     printf("DEBUG: Frame with %d circle-circle contacts detected\n", circleContactCount);
    // }

    // Initialize and warmstart forces
    for (Force* force = forces; force != 0;)
    {
        // Initialization can including caching anything that is constant over the step
        if (!force->initialize())
        {
            // Force has returned false meaning it is inactive, so remove it from the solver
            Force* next = force->next;
            delete force;
            force = next;
        }
        else
        {
            for (int i = 0; i < force->rows(); i++)
            {
                if (postStabilize)
                {
                    // With post stabilization, we can reuse the full lambda from the previous step,
                    // and only need to reduce the penalty parameters
                    force->penalty[i] = clamp(force->penalty[i] * gamma, PENALTY_MIN, PENALTY_MAX);
                }
                else
                {
                    // Warmstart the dual variables and penalty parameters (Eq. 19)
                    // Penalty is safely clamped to a minimum and maximum value
                    force->lambda[i] = force->lambda[i] * alpha * gamma;
                    force->penalty[i] = clamp(force->penalty[i] * gamma, PENALTY_MIN, PENALTY_MAX);
                }

                // If it's not a hard constraint, we don't let the penalty exceed the material stiffness
                force->penalty[i] = min(force->penalty[i], force->stiffness[i]);
            }

            force = force->next;
        }
    }

    // Initialize and warmstart bodies (ie primal variables)
    for (Rigid* body = bodies; body != 0; body = body->next)
    {
        // Don't let bodies rotate too fast
        body->velocity.z = clamp(body->velocity.z, -10.0f, 10.0f);

        // Compute inertial position (Eq 2)
        body->inertial = body->position + body->velocity * dt;
        if (body->mass > 0)
            body->inertial += float3{ 0, gravity, 0 } * (dt * dt);

        // Adaptive warmstart (See original VBD paper)
        float3 accel = (body->velocity - body->prevVelocity) / dt;
        float accelExt = accel.y * sign(gravity);
        float accelWeight = clamp(accelExt / abs(gravity), 0.0f, 1.0f);
        if (!isfinite(accelWeight)) accelWeight = 0.0f;

        // Save initial position (x-) and compute warmstarted position (See original VBD paper)
        body->initial = body->position;
        body->position = body->position + body->velocity * dt + float3{ 0, gravity, 0 } * (accelWeight * dt * dt);
    }

    // Main solver loop
    // If using post stabilization, we'll use one extra iteration for the stabilization
    int totalIterations = iterations + (postStabilize ? 1 : 0);

    for (int it = 0; it < totalIterations; it++)
    {
        // If using post stabilization, either remove all or none of the pre-existing constraint error
        float currentAlpha = alpha;
        if (postStabilize)
            currentAlpha = it < iterations ? 1.0f : 0.0f;

        // Primal update
        for (Rigid* body = bodies; body != 0; body = body->next)
        {
            // Skip static / kinematic bodies
            if (body->mass <= 0)
                continue;

            // Initialize left and right hand sides of the linear system (Eqs. 5, 6)
            float3x3 M = diagonal(body->mass, body->mass, body->moment);
            float3x3 lhs = M / (dt * dt);
            float3 rhs = M / (dt * dt) * (body->position - body->inertial);

            // Iterate over all forces acting on the body
            for (Force* force = body->forces; force != 0; force = (force->bodyA == body) ? force->nextA : force->nextB)
            {
                // Compute constraint and its derivatives
                force->computeConstraint(currentAlpha);
                force->computeDerivatives(body);

                for (int i = 0; i < force->rows(); i++)
                {
                    // Use lambda as 0 if it's not a hard constraint
                    float lambda = isinf(force->stiffness[i]) ? force->lambda[i] : 0.0f;

                    // Compute the clamped force magnitude (Sec 3.2)
                    float f = clamp(force->penalty[i] * force->C[i] + lambda + force->motor[i], force->fmin[i], force->fmax[i]);

                    // Compute the diagonally lumped geometric stiffness term (Sec 3.5)
                    float3x3 G = diagonal(length(force->H[i].col(0)), length(force->H[i].col(1)), length(force->H[i].col(2))) * abs(f);

                    // Accumulate force (Eq. 13) and hessian (Eq. 17)
                    rhs += force->J[i] * f;
                    lhs += outer(force->J[i], force->J[i] * force->penalty[i]) + G;

                    // DEBUG: Track forces on circles
                    // if (body->shapeType == SHAPE_CIRCLE && abs(f) > 0.1f)
                    // {
                    //     printf("DEBUG: Circle at (%.3f, %.3f) force: %.3f, constraint: %.3f, penalty: %.3f\n", 
                    //            body->position.x, body->position.y, f, force->C[i], force->penalty[i]);
                    //     printf("DEBUG: Force direction: (%.3f, %.3f, %.3f)\n", force->J[i].x, force->J[i].y, force->J[i].z);
                    //     
                    //     // Track if this is a tangential force (odd indices are tangential)
                    //     if (i % 2 == 1) {
                    //         printf("DEBUG: TANGENTIAL force on circle: %.3f\n", f);
                    //     }
                    // }
                }
            }

            // Solve the SPD linear system using LDL and apply the update (Eq. 4)
            float3 update = solve(lhs, rhs);
            body->position -= update;

            // DEBUG: Track orientation changes for circles
            // if (body->shapeType == SHAPE_CIRCLE && abs(update.z) > 0.01f)
            // {
            //     printf("DEBUG: Circle orientation change: %.3f radians (%.1f degrees)\n", update.z, update.z * 180.0f / 3.14159f);
            //     printf("DEBUG: Circle new orientation: %.3f radians (%.1f degrees)\n", body->position.z, body->position.z * 180.0f / 3.14159f);
            // }
        }

        // Dual update, only for non stabilized iterations in the case of post stabilization
        // If doing more than one post stabilization iteration, we can still do a dual update,
        // but make sure not to persist the penalty or lambda updates done during the stabilization iterations for the next frame.
        if (it < iterations)
        {
            for (Force* force = forces; force != 0; force = force->next)
            {
                // Compute constraint
                force->computeConstraint(currentAlpha);

                for (int i = 0; i < force->rows(); i++)
                {
                    // Use lambda as 0 if it's not a hard constraint
                    float lambda = isinf(force->stiffness[i]) ? force->lambda[i] : 0.0f;

                    // Update lambda (Eq 11)
                    // Note that we don't include non-conservative forces (ie motors) in the lambda update, as they are not part of the dual problem.
                    force->lambda[i] = clamp(force->penalty[i] * force->C[i] + lambda, force->fmin[i], force->fmax[i]);

                    // Disable the force if it has exceeded its fracture threshold
                    if (fabsf(force->lambda[i]) >= force->fracture[i])
                        force->disable();

                    // Update the penalty parameter and clamp to material stiffness if we are within the force bounds (Eq. 16)
                    if (force->lambda[i] > force->fmin[i] && force->lambda[i] < force->fmax[i])
                        force->penalty[i] = min(force->penalty[i] + beta * abs(force->C[i]), min(PENALTY_MAX, force->stiffness[i]));
                }
            }
        }

        // If we are are the final iteration before post stabilization, compute velocities (BDF1)
        if (it == iterations - 1)
        {
            for (Rigid* body = bodies; body != 0; body = body->next)
            {
                body->prevVelocity = body->velocity;
                if (body->mass > 0)
                    body->velocity = (body->position - body->initial) / dt;
                
                // TEMPORARY FIX: Apply angular damping to circles to prevent energy accumulation
                // from numerical instabilities during sustained contact
                if (body->shapeType == SHAPE_CIRCLE) {
                    float angularDamping = 0.50f;  // User's preferred value
                    body->velocity.z *= angularDamping;
                }
            }
        }

        // If using post-stabilization, apply a final pass of position-based corrections.
        if (postStabilize && it == iterations)
        {
            currentAlpha = 1.0f; // No velocity interpolation
        }
    }
    
    // DISABLED: Using static wall bodies instead of post-process boundary handling
    // handleBoundaryCollisions();
}

void Solver::handleBoundaryCollisions()
{
    for (Rigid* body = bodies; body != 0; body = body->next)
    {
        if (body->mass <= 0) continue; // Skip static bodies
        
        float2 pos = body->position.xy();
        float2 vel = body->velocity.xy();
        bool collided = false;
        
        // For circles, use the radius; for boxes, use half the size
        float bodyRadius = (body->shapeType == SHAPE_CIRCLE) ? body->size.x : length(body->size * 0.5f);
        
        // Left boundary - only act if penetrating AND moving toward boundary
        float leftPenetration = boundaryLeft + bodyRadius - pos.x;
        if (leftPenetration > 0.01f && vel.x < -0.01f)
        {
            body->position.x = boundaryLeft + bodyRadius;
            body->velocity.x = -vel.x * boundaryRestitution;
            collided = true;
        }
        
        // Right boundary - only act if penetrating AND moving toward boundary
        float rightPenetration = pos.x + bodyRadius - boundaryRight;
        if (rightPenetration > 0.01f && vel.x > 0.01f)
        {
            body->position.x = boundaryRight - bodyRadius;
            body->velocity.x = -vel.x * boundaryRestitution;
            collided = true;
        }
        
        // Bottom boundary - only act if penetrating AND moving toward boundary
        float penetration = boundaryBottom + bodyRadius - pos.y;
        if (penetration > 0.01f && vel.y < -0.01f) // Only if moving downward with significant velocity
        {
            body->position.y = boundaryBottom + bodyRadius;
            body->velocity.y = -vel.y * boundaryRestitution;
            collided = true;
        }
        
        // Top boundary - only act if penetrating AND moving toward boundary
        float topPenetration = pos.y + bodyRadius - boundaryTop;
        if (topPenetration > 0.01f && vel.y > 0.01f)
        {
            body->position.y = boundaryTop - bodyRadius;
            body->velocity.y = -vel.y * boundaryRestitution;
            collided = true;
        }
        
        // Light friction on collision to make it more realistic
        if (collided)
        {
            body->velocity.x *= 0.98f; // Very light friction
            body->velocity.z *= 0.95f; // Light rotation damping
        }
    }
}

void Solver::updateBoundaries(float left, float right, float bottom, float top)
{
    boundaryLeft = left;
    boundaryRight = right;
    boundaryBottom = bottom;
    boundaryTop = top;
}

void Solver::draw()
{
    for (Rigid* body = bodies; body != 0; body = body->next)
        body->draw();
    for (Force* force = forces; force != 0; force = force->next)
        force->draw();
}
