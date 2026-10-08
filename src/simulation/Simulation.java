package simulation;

import math.Vector2;
import physics.Body;
import physics.Collision;
import physics.CollisionDetector;
import physics.ContinuousCollision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Set;

public class Simulation {
    // how many times the velocity solver sweeps over all collisions each step.
    // Weight has to travel down a stack one contact per sweep, so tall stacks need many sweeps
    // (a stack of 10 boxes only stays up at around 80). Lower it if you have many bodies and no tall stacks.
    private static final int SOLVER_ITERATIONS = 80;

    private final int fps;

    private final Vector2 gravitationalAcceleration = new Vector2(0, -9.81f);
    private final float airDensity = 1.225f; // kg/m^3   for now we'll use F = -airdensity * speed²

    private final List<Body> bodies = new ArrayList<>();

    // continuous collision detection: stops fast bodies from skipping over thin ones between two frames
    private boolean continuousCollisionEnabled = true;

    /** Two bodies in the order they appear in the body list (identity based equals/hashCode). */
    private record BodyPair(Body a, Body b) {}

    /** Two bodies that hit each other at the given fraction of the step. */
    private record Impact(Body a, Body b, float time) {}

    public Simulation(int fps) {
        this.fps = fps;
    }

    private void updateBody(Body body, float dt) {
        if (body == null) return;

        Vector2 gravitationalForce = getGravitationalForce(body);
        Vector2 dragForce = body.getVelocity().mult(-airDensity);

        body.applyForces(gravitationalForce, dragForce);

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

        // 2. continuous collision detection: bodies that would have skipped over each other are moved back
        //    to the moment they touched
        Set<BodyPair> rewoundPairs = continuousCollisionEnabled ? applyContinuousCollision() : Set.of();

        // 3. detect collisions
        List<Collision> collisions = detectCollisions(rewoundPairs);

        // 4. solve velocities: prepare once, then sweep over all collisions several times.
        //    Every other sweep runs backwards so the result doesn't favour whichever pair happens to come first.
        for (Collision collision : collisions) {
            collision.prepare();
        }

        for (int i = 0; i < SOLVER_ITERATIONS; i++) {
            if (i % 2 == 0) {
                for (int c = 0; c < collisions.size(); c++) {
                    collisions.get(c).resolveVelocities();
                }
            } else {
                for (int c = collisions.size() - 1; c >= 0; c--) {
                    collisions.get(c).resolveVelocities();
                }
            }
        }

        // 5. snap barely-moving bodies in stable contact to rest
        for (Collision collision : collisions) {
            collision.settleBodies();
        }

        // 6. push overlapping bodies apart, once
        for (Collision collision : collisions) {
            collision.correctPositions();
        }
    }

    /**
     * Finds pairs that hit each other during this step and moves them back to the time of impact.
     * Impacts are handled earliest first. A body is only moved back once per step: later impacts that involve
     * it are skipped (they are caught by the normal detection or by the next step).
     *
     * @return the pairs that were moved back; they must be detected with a small margin (see detectCollisions)
     */
    private Set<BodyPair> applyContinuousCollision() {
        List<Impact> impacts = new ArrayList<>();

        for (int i = 0; i < bodies.size(); i++) {
            Body current = bodies.get(i);
            if (current == null || current.getCollider() == null) continue;

            for (int j = i + 1; j < bodies.size(); j++) {
                Body other = bodies.get(j);
                if (other == null || other.getCollider() == null) continue;

                if (current.getInverseMass() == 0 && other.getInverseMass() == 0) continue;

                float time = ContinuousCollision.timeOfImpact(current, other);
                if (time >= 0f) {
                    impacts.add(new Impact(current, other, time));
                }
            }
        }

        impacts.sort(Comparator.comparingDouble(Impact::time));

        Set<Body> movedBack = Collections.newSetFromMap(new IdentityHashMap<>());
        Set<BodyPair> pairs = new HashSet<>();

        for (Impact impact : impacts) {
            boolean aMovable = impact.a().getInverseMass() != 0;
            boolean bMovable = impact.b().getInverseMass() != 0;

            if ((aMovable && movedBack.contains(impact.a())) || (bMovable && movedBack.contains(impact.b()))) {
                continue;
            }

            if (aMovable) {
                impact.a().moveToFractionOfLastStep(impact.time());
                movedBack.add(impact.a());
            }
            if (bMovable) {
                impact.b().moveToFractionOfLastStep(impact.time());
                movedBack.add(impact.b());
            }

            pairs.add(new BodyPair(impact.a(), impact.b()));
        }

        return pairs;
    }

    private List<Collision> detectCollisions(Set<BodyPair> rewoundPairs) {
        List<Collision> collisions = new ArrayList<>();

        for (int i = 0; i < bodies.size(); i++) {
            Body current = bodies.get(i);
            if (current == null || current.getCollider() == null) continue;

            for (int j = i + 1; j < bodies.size(); j++) {
                Body other = bodies.get(j);
                if (other == null || other.getCollider() == null) continue;

                // two immovable bodies can never react to each other
                if (current.getInverseMass() == 0 && other.getInverseMass() == 0) continue;

                // a rewound pair sits at the moment of impact: touching, but not overlapping yet
                float margin = rewoundPairs.contains(new BodyPair(current, other)) ? ContinuousCollision.MARGIN : 0f;

                Collision collision = CollisionDetector.detectCollision(current.getCollider(), other.getCollider(), margin);
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

    public boolean isContinuousCollisionEnabled() {
        return continuousCollisionEnabled;
    }

    public void setContinuousCollisionEnabled(boolean enabled) {
        this.continuousCollisionEnabled = enabled;
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