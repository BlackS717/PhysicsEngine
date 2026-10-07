package simulation;

import math.Vector2;
import physics.Body;
import physics.Collision;
import physics.CollisionDetector;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Simulation {
    // how many times the velocity solver sweeps over all collisions each step.
    // More passes = more stable resting contacts and stacks, at a small cost.
    private static final int SOLVER_ITERATIONS = 4;

    private final int fps;

    private final Vector2 gravitationalAcceleration = new Vector2(0, -9.81f);

    private final List<Body> bodies = new ArrayList<>();

    public Simulation(int fps) {
        this.fps = fps;
    }

    private void updateBody(Body body, float dt) {
        if (body == null) return;

        Vector2 gravitationalForce = getGravitationalForce(body);

        body.applyForces(gravitationalForce);

        body.integrate(dt);
    }

    private Vector2 getGravitationalForce(Body body) {
        if (body == null) throw new NullPointerException();

        return gravitationalAcceleration.mult(body.getMass());
    }

    public void step() {
        float dt = 1.0f / fps;

        // 1. integrate all bodies
        for (Body body : bodies) {
            updateBody(body, dt);
        }

        // 2. detect collisions
        List<Collision> collisions = detectCollisions();

        // 3. resolve velocities several times: one pass can't satisfy every contact at once
        for (int i = 0; i < SOLVER_ITERATIONS; i++) {
            for (Collision collision : collisions) {
                collision.resolveVelocities();
            }
        }

        // 4. push overlapping bodies apart, once
        for (Collision collision : collisions) {
            collision.correctPositions();
        }
    }

    private List<Collision> detectCollisions() {
        List<Collision> collisions = new ArrayList<>();

        for (int i = 0; i < bodies.size(); i++) {
            Body current = bodies.get(i);
            if (current == null || current.getCollider() == null) continue;

            for (int j = i + 1; j < bodies.size(); j++) {
                Body other = bodies.get(j);
                if (other == null || other.getCollider() == null) continue;

                // two immovable bodies can never react to each other
                if (current.getInverseMass() == 0 && other.getInverseMass() == 0) continue;

                Collision collision = CollisionDetector.detectCollision(current.getCollider(), other.getCollider());
                if (collision != null) {
                    collisions.add(collision);
                }
            }
        }

        return collisions;
    }

    public void reset() {
        for (Body body : bodies) {
            body.reset();
        }
    }

    public int getFps() {
        return this.fps;
    }

    public List<Body> getBodies() {
        return Collections.unmodifiableList(this.bodies);
    }

    public void addBody(Body body) {
        this.bodies.add(body);
    }

    public void removeBody(Body body) {
        this.bodies.remove(body);
    }
}