package physics;

import math.Vector2;

// this class represents a box collider with circle collider as corners
// it will be used to generate different structures like chains, or any other structure that can be represented as a series of segments
// any Body can be attached to the circle colliders at the corners of the box collider,

/**
 * A rigid link of fixed length between two existing bodies.
 *
 * - The two end bodies are the "corners": each gets a CircleCollider (radius = thickness / 2) unless it already
 *   has a collider, so a body shared by two segments (a chain) is only given one.
 * - A BoxCollider (length x thickness) between the two ends, owned by an internal SegmentBody, forms a capsule
 *   together with the corner circles. It is placed at the midpoint of the ends and turned along the segment.
 * - A hit on the box is shared between the two end bodies by where it landed along the segment
 *   (see CollisionSide), so the segment pushes both of them.
 * - The distance between the two ends is kept equal to the length by a constraint that Simulation solves
 *   (solveVelocityConstraint / solvePositionConstraint). The ends can still swing and spin freely around each
 *   other, which is what makes chains and ropes out of several segments.
 *
 * Add it with Simulation.addSegment(...), not addBody(...).
 */
public class Segment {
    private static final float MIN_AXIS_LENGTH = 1e-6f;

    private final Body endA;
    private final Body endB;
    private final float length;
    private final float thickness;
    private final SegmentBody middle;

    /** The rest length is the distance the two bodies currently have. */
    public Segment(Body endA, Body endB, float thickness) {
        this(endA, endB, distanceBetween(endA, endB), thickness);
    }

    /** If the bodies are not exactly `length` apart, the constraint pulls them to that distance. */
    public Segment(Body endA, Body endB, float length, float thickness) {
        if (endA == null || endB == null) {
            throw new IllegalArgumentException("A segment needs two bodies.");
        }
        if (endA == endB) {
            throw new IllegalArgumentException("A segment needs two different bodies.");
        }
        if (length <= 0 || thickness <= 0) {
            throw new IllegalArgumentException("Length and thickness must be positive.");
        }

        this.endA = endA;
        this.endB = endB;
        this.length = length;
        this.thickness = thickness;

        // circle colliders at the corners (a body that already has a collider keeps it)
        attachCorner(endA);
        attachCorner(endB);

        // box collider between them, owned by an internal body
        this.middle = new SegmentBody(endA, endB);
        this.middle.setCollider(new BoxCollider(middle, length, thickness));
        this.middle.setRestitution((endA.getRestitution() + endB.getRestitution()) / 2f);
        this.middle.setStaticFriction((endA.getStaticFriction() + endB.getStaticFriction()) / 2f);
        this.middle.setDynamicFriction((endA.getDynamicFriction() + endB.getDynamicFriction()) / 2f);
        this.middle.getRenderInfo().setColor(endA.getRenderInfo().getColor());

        updateMiddleBody();
    }

    private void attachCorner(Body body) {
        if (body.getCollider() == null) {
            body.setCollider(new CircleCollider(body, thickness / 2f));
        }
    }

    private static float distanceBetween(Body a, Body b) {
        if (a == null || b == null) {
            throw new IllegalArgumentException("A segment needs two bodies.");
        }
        return b.getTransform().getPosition().sub(a.getTransform().getPosition()).magnitude();
    }

    // ---------------------------------------------------------------- constraint (solved by Simulation)

    /** Removes the speed at which the two ends move toward or away from each other along the segment. */
    public void solveVelocityConstraint() {
        float invMassA = endA.getInverseMass();
        float invMassB = endB.getInverseMass();
        float inverseMassSum = invMassA + invMassB;
        if (inverseMassSum == 0) return; // both ends immovable

        Vector2 delta = endB.getTransform().getPosition().sub(endA.getTransform().getPosition());
        float distance = delta.magnitude();
        if (distance < MIN_AXIS_LENGTH) return;

        Vector2 axis = delta.div(distance); // from A to B

        float relativeSpeed = endB.getVelocity().sub(endA.getVelocity()).dotProduct(axis);
        float impulse = -relativeSpeed / inverseMassSum;

        endA.setVelocity(endA.getVelocity().sub(axis.mult(impulse * invMassA)));
        endB.setVelocity(endB.getVelocity().add(axis.mult(impulse * invMassB)));
    }

    /** Moves the ends so that they are exactly `length` apart again (the lighter end moves more). */
    public void solvePositionConstraint() {
        float invMassA = endA.getInverseMass();
        float invMassB = endB.getInverseMass();
        float inverseMassSum = invMassA + invMassB;
        if (inverseMassSum == 0) return;

        Vector2 posA = endA.getTransform().getPosition();
        Vector2 posB = endB.getTransform().getPosition();

        Vector2 delta = posB.sub(posA);
        float distance = delta.magnitude();
        if (distance < MIN_AXIS_LENGTH) return;

        Vector2 axis = delta.div(distance);
        float error = distance - length; // > 0: stretched, < 0: compressed

        Vector2 correction = axis.mult(error / inverseMassSum);

        endA.getTransform().setPosition(posA.add(correction.mult(invMassA)));
        endB.getTransform().setPosition(posB.sub(correction.mult(invMassB)));
    }

    // ---------------------------------------------------------------- middle box

    /**
     * Places the middle body at the midpoint of the ends, turned along the segment, and gives it the pose it had
     * at the start of the step (needed by continuous collision detection). Called by Simulation.
     */
    public void updateMiddleBody() {
        Vector2 a = endA.getTransform().getPosition();
        Vector2 b = endB.getTransform().getPosition();

        float angle = angleOf(a, b, middle.getTransform().getRotation());

        middle.getTransform().setPosition(a.add(b).mult(0.5f));
        middle.getTransform().setRotation(angle);

        // pose at the start of the step, from where the ends were then (immovable ends never moved)
        Vector2 previousA = endA.isStatic() ? a : endA.getPreviousPosition();
        Vector2 previousB = endB.isStatic() ? b : endB.getPreviousPosition();

        float previousAngle = angleOf(previousA, previousB, angle);
        // keep the two angles within half a turn so interpolating between them takes the short way round
        while (angle - previousAngle > Math.PI) previousAngle += (float) (2 * Math.PI);
        while (angle - previousAngle < -Math.PI) previousAngle -= (float) (2 * Math.PI);

        middle.setPreviousPose(previousA.add(previousB).mult(0.5f), previousAngle);
    }

    private static float angleOf(Vector2 from, Vector2 to, float fallback) {
        Vector2 d = to.sub(from);
        if (d.magnitude() < MIN_AXIS_LENGTH) return fallback;
        return (float) Math.atan2(d.getY(), d.getX());
    }

    // ---------------------------------------------------------------- getters

    public Body getEndA() {
        return endA;
    }

    public Body getEndB() {
        return endB;
    }

    /** The body that owns the box collider. Use it to change the segment's colour, friction or restitution. */
    public SegmentBody getMiddleBody() {
        return middle;
    }

    public float getLength() {
        return length;
    }

    public float getThickness() {
        return thickness;
    }

    /** True if the two segments have an end body in common (they are neighbours in a chain). */
    public boolean sharesEndWith(Segment other) {
        return endA == other.endA || endA == other.endB || endB == other.endA || endB == other.endB;
    }
}