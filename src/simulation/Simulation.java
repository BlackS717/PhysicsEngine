package simulation;

import math.Vector2;
import physics.Body;
import physics.Collision;
import physics.CollisionDetector;
import physics.ContinuousCollision;
import physics.Segment;
import physics.SegmentBody;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Set;

public class Simulation {
    // how many times the velocity solver sweeps over all collisions each step.
    // Weight has to travel down a stack (or along a chain) one contact per sweep, so tall stacks need many sweeps
    // (a stack of 10 boxes only stays up at around 80). Lower it if you have many bodies and no tall stacks.
    private static final int SOLVER_ITERATIONS = 80;

    // sweeps that pull segment ends back to their exact length after the velocities are solved
    private static final int SEGMENT_POSITION_ITERATIONS = 30;

    private final int fps;

    private final Vector2 gravitationalAcceleration = new Vector2(0, -9.81f);

    private final List<Body> bodies = new ArrayList<>();
    private final List<Segment> segments = new ArrayList<>();

    // pairs that must never collide (the parts of one segment, neighbouring segments in a chain)
    private final Set<BodyPair> ignoredPairs = new HashSet<>();

    // continuous collision detection: stops fast bodies from skipping over thin ones between two frames
    private boolean continuousCollisionEnabled = true;

    /** Two bodies (identity based equals/hashCode). */
    private record BodyPair(Body a, Body b) {}

    /** Two bodies that hit each other at the given fraction of the step. */
    private record Impact(Body a, Body b, float time) {}

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

        // 1. integrate all bodies (the middle bodies of segments don't move by themselves)
        for (Body body : bodies) {
            updateBody(body, dt);
        }
        updateSegments();

        // 2. continuous collision detection: bodies that would have skipped over each other are moved back
        //    to the moment they touched
        Set<BodyPair> rewoundPairs = continuousCollisionEnabled ? applyContinuousCollision() : Set.of();
        updateSegments();

        // 3. detect collisions
        List<Collision> collisions = detectCollisions(rewoundPairs);

        // 4. solve velocities: prepare once, then sweep over all collisions (and segment constraints) several times.
        //    Every other sweep runs backwards so the result doesn't favour whichever pair happens to come first.
        for (Collision collision : collisions) {
            collision.prepare();
        }

        for (int i = 0; i < SOLVER_ITERATIONS; i++) {
            for (Segment segment : segments) {
                segment.solveVelocityConstraint();
            }

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

        // 7. bring every segment back to its exact length, then place its middle box
        for (int i = 0; i < SEGMENT_POSITION_ITERATIONS; i++) {
            for (Segment segment : segments) {
                segment.solvePositionConstraint();
            }
        }
        updateSegments();
    }

    private void updateSegments() {
        for (Segment segment : segments) {
            segment.updateMiddleBody();
        }
    }

    /**
     * Finds pairs that hit each other during this step and moves them back to the time of impact.
     * Impacts are handled earliest first. A body is only moved back once per step: later impacts that involve
     * it are skipped (they are caught by the normal detection or by the next step).
     * The middle box of a segment is moved back by moving back its two end bodies.
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
                if (isIgnored(current, other)) continue;

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
            List<Body> movers = new ArrayList<>();
            addMovableBodies(impact.a(), movers);
            addMovableBodies(impact.b(), movers);

            boolean alreadyMoved = false;
            for (Body mover : movers) {
                if (movedBack.contains(mover)) alreadyMoved = true;
            }
            if (alreadyMoved) continue;

            for (Body mover : movers) {
                mover.moveToFractionOfLastStep(impact.time());
                movedBack.add(mover);
            }

            pairs.add(new BodyPair(impact.a(), impact.b()));
        }

        return pairs;
    }

    /** The bodies that really have to be moved to move `body`: itself, or both ends if it's a segment's middle. */
    private void addMovableBodies(Body body, List<Body> into) {
        if (body instanceof SegmentBody segmentBody) {
            addMovableBodies(segmentBody.getEndA(), into);
            addMovableBodies(segmentBody.getEndB(), into);
        } else if (body.getInverseMass() != 0 && !into.contains(body)) {
            into.add(body);
        }
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
                if (isIgnored(current, other)) continue;

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

    private boolean isIgnored(Body a, Body b) {
        return !ignoredPairs.isEmpty() && ignoredPairs.contains(new BodyPair(a, b));
    }

    private void ignoreCollisions(Body a, Body b) {
        ignoredPairs.add(new BodyPair(a, b));
        ignoredPairs.add(new BodyPair(b, a));
    }

    public void reset() {
        for (Body body : bodies) {
            body.reset();
        }
        updateSegments();
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

    /**
     * Adds a segment: its two end bodies (if they aren't in the simulation yet) and its middle box.
     * The parts of one segment never collide with each other, and neither do the boxes of two segments that
     * share an end body (neighbours in a chain).
     */
    public void addSegment(Segment segment) {
        if (segments.contains(segment)) return;

        if (!bodies.contains(segment.getEndA())) bodies.add(segment.getEndA());
        if (!bodies.contains(segment.getEndB())) bodies.add(segment.getEndB());
        bodies.add(segment.getMiddleBody());

        ignoreCollisions(segment.getEndA(), segment.getEndB());
        ignoreCollisions(segment.getMiddleBody(), segment.getEndA());
        ignoreCollisions(segment.getMiddleBody(), segment.getEndB());

        for (Segment other : segments) {
            if (segment.sharesEndWith(other)) {
                ignoreCollisions(segment.getMiddleBody(), other.getMiddleBody());
            }
        }

        segments.add(segment);
        segment.updateMiddleBody();
    }

    public void removeSegment(Segment segment) {
        if (!segments.remove(segment)) return;

        bodies.remove(segment.getMiddleBody());
        ignoredPairs.removeIf(pair -> pair.a() == segment.getMiddleBody() || pair.b() == segment.getMiddleBody());
        ignoredPairs.remove(new BodyPair(segment.getEndA(), segment.getEndB()));
        ignoredPairs.remove(new BodyPair(segment.getEndB(), segment.getEndA()));
    }

    public List<Segment> getSegments() {
        return Collections.unmodifiableList(this.segments);
    }
}