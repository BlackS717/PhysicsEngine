package physics;

import math.Transform;
import math.Vector2;

/**
 * The middle part of a Segment: a body that owns the segment's BoxCollider.
 *
 * It has no motion of its own. Segment.updateMiddleBody() places it at the midpoint of the two end bodies and
 * turns it along the segment every step, and a hit on it is pushed on to the two end bodies by the collision
 * solver (see CollisionSide), so it never needs to be integrated.
 */
public class SegmentBody extends Body {
    private final Body endA;
    private final Body endB;

    SegmentBody(Body endA, Body endB) {
        super(new Transform(new Vector2(0, 0)), endA.getMass() + endB.getMass());
        this.endA = endA;
        this.endB = endB;
    }

    public Body getEndA() {
        return endA;
    }

    public Body getEndB() {
        return endB;
    }

    // nothing here moves by itself

    @Override
    public void integrate(float dt) {
    }

    @Override
    public void applyForces(Vector2... forces) {
    }

    @Override
    public void moveToFractionOfLastStep(float t) {
    }

    @Override
    public void reset() {
        // follows its end bodies: Segment.updateMiddleBody() repositions it
    }

    /** Immovable only if both ends are; this is what the simulation uses to skip pairs that can't react. */
    @Override
    public float getInverseMass() {
        return endA.getInverseMass() + endB.getInverseMass();
    }

    @Override
    public float getInverseInertia() {
        return 0f;
    }
}