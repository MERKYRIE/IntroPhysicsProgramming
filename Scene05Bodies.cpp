#include "Scene05Bodies.hpp"
#include "SphereCube.hpp"
#include "ShapeSphere.hpp"
#include "Intersections.hpp"
#include "Contact.hpp"
#include "Broadphase.hpp"

using gphysics::ShapeSphere;
using gphysics::Intersections;
using gphysics::Contact;
using gphysics::CollisionPair;

void Scene05Bodies::Load(Renderer& renderer) {
    SphereCube * sphere;
    /*
    sphere = new SphereCube(renderer);
    sphere->Load();
    bodies.emplace_back(Vec(0, 0, 0), Quat::identity, new ShapeSphere(1), sphere);
    bodies.back().linearVelocity = Vec::zero;
    bodies.back().inverseMass = 1.0f;
    bodies.back().elasticity = 0.5f;
    bodies.back().angularVelocity = Vec::zero;
    bodies.back().friction = 0.5f;
    */
    sphere = new SphereCube(renderer);
    sphere->Load();
    bodies.emplace_back(Vec(0, -5, -5), Quat::identity, new ShapeSphere(1000), sphere);
    bodies.back().linearVelocity = Vec::zero;
    bodies.back().inverseMass = 0.0f;
    bodies.back().elasticity = 0.99f;
    bodies.back().angularVelocity = Vec::zero;
    bodies.back().friction = 0.5f;
}

bool Scene05Bodies::Update(float dt) {
    bool stillRunning = ManageInput(inputState);

    // Gravity
    for (int i = 0; i < bodies.size(); ++i)
    {
        Body& body = bodies[i];
        float mass = 1.0f / body.inverseMass;
        // Gravity needs to be an impulse I
        // I == dp, so F == dp/dt <=> dp = F * dt
        // <=> I = F * dt <=> I = m * g * dt
        Vec impulseGravity = Vec(0, -10, 0) * mass * dt;
        body.ApplyImpulseLinear(impulseGravity);
    }

    // Broadphase
    std::vector<CollisionPair> collisionPairs;
    BroadPhase(bodies.data(), bodies.size(), collisionPairs, dt);

    // Collision checks (Narrow phase)
    int numContacts = 0;
    const int maxContacts = bodies.size() * bodies.size();
    Contact* contacts = (Contact*)alloca(sizeof(Contact) * maxContacts);
    for (int i = 0; i < collisionPairs.size(); ++i)
    {
        const CollisionPair& pair = collisionPairs[i];
        Body& bodyA = bodies[pair.a];
        Body& bodyB = bodies[pair.b];

        if (bodyA.inverseMass == 0.0f && bodyB.inverseMass == 0.0f)
            continue;

        Contact contact;
        if (Intersections::Intersect(bodyA, bodyB, dt, contact))
        {
            contacts[numContacts] = contact;
            ++numContacts;
        }
    }

    // Sort times of impact
    if (numContacts > 1) {
        qsort(contacts, numContacts, sizeof(Contact),
            Contact::CompareContact);
    }

    // Contact resolve in order
    float accumulatedTime = 0.0f;
    for (int i = 0; i < numContacts; ++i)
    {
        Contact& contact = contacts[i];
        const float dt = contact.timeOfImpact - accumulatedTime;
        Body* bodyA = contact.a;
        Body* bodyB = contact.b;

        // Skip body par with infinite mass
        if (bodyA->inverseMass == 0.0f && bodyB->inverseMass == 0.0f)
            continue;

        // Position update
        for (int j = 0; j < bodies.size(); ++j) {
            bodies[j].Update(dt);
        }

        Contact::ResolveContact(contact);
        accumulatedTime += dt;
    }

    // Other physics behavirous, outside collisions.
    // Update the positions for the rest of this frame's time.
    const float timeRemaining = dt - accumulatedTime;
    if (timeRemaining > 0.0f)
    {
        for (int i = 0; i < bodies.size(); ++i) {
            bodies[i].Update(timeRemaining);
        }
    }

    return stillRunning;
}

void Scene05Bodies::Draw(Renderer& renderer) {
    renderer.Begin();

    for (auto& body : bodies) {
        body.drawable->Draw(renderer);
    }
    renderer.End();
}

void Scene05Bodies::Unload(Renderer& renderer) {
    for (auto& body : bodies) {
        body.drawable->Unload();
    }
}