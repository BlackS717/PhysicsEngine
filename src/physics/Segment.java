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
 *   (prepare / solveVelocityConstraint / solvePositionConstraint). The ends can still swing and spin freely around
 *   each other, which is what makes chains and ropes out of several segments.
 * - Rigidity and elasticity (see SoftConstraint) turn that constraint into a spring. With rigidity 1 (the default)
 *   the length is fixed. Lower it and the segment can be stretched and compressed, and pulls back to its length.
 *   A segment created with a length different from the current distance starts out loaded like a spring.
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

    private final SoftConstraint spring = new SoftConstraint();

    // per-step data, filled in by prepare()
    private boolean active;
    private Vector2 axis;
    private float inverseMassSum;

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

    /** Call once per step before the solver iterations: measures the segment and sets up the spring. */
    public void prepare(float dt) {
        active = false;

        float inverseMassSum = endA.getInverseMass() + endB.getInverseMass();
        if (inverseMassSum == 0) return; // both ends immovable

        Vector2 delta = endB.getTransform().getPosition().sub(endA.getTransform().getPosition());
        float distance = delta.magnitude();
        if (distance < MIN_AXIS_LENGTH) return;

        this.axis = delta.div(distance); // from A to B
        this.inverseMassSum = inverseMassSum;
        this.active = true;

        spring.prepare(1f / inverseMassSum, distance - length, dt);
    }

    /**
     * One solver iteration on the speed at which the two ends move toward or away from each other.
     * Rigid: that speed is removed. Soft: it is pulled toward the speed that brings the length back.
     */
    public void solveVelocityConstraint() {
        if (!active) return;

        float relativeSpeed = endB.getVelocity().sub(endA.getVelocity()).dotProduct(axis);
        float impulse = spring.solve(relativeSpeed, inverseMassSum);

        endA.setVelocity(endA.getVelocity().sub(axis.mult(impulse * endA.getInverseMass())));
        endB.setVelocity(endB.getVelocity().add(axis.mult(impulse * endB.getInverseMass())));
    }

    /**
     * Moves the ends so that they are exactly `length` apart again (the lighter end moves more).
     * Only for rigid segments: a soft one is allowed to stay stretched, its spring pulls it back.
     */
    public void solvePositionConstraint() {
        if (!spring.isRigid()) return;

        float invMassA = endA.getInverseMass();
        float invMassB = endB.getInverseMass();
        float inverseMassSum = invMassA + invMassB;
        if (inverseMassSum == 0) return;

        Vector2 posA = endA.getTransform().getPosition();
        Vector2 posB = endB.getTransform().getPosition();

        Vector2 delta = posB.sub(posA);
        float distance = delta.magnitude();
        if (distance < MIN_AXIS_LENGTH) return;

        Vector2 direction = delta.div(distance);
        float error = distance - length; // > 0: stretched, < 0: compressed

        Vector2 correction = direction.mult(error / inverseMassSum);

        endA.getTransform().setPosition(posA.add(correction.mult(invMassA)));
        endB.getTransform().setPosition(posB.sub(correction.mult(invMassB)));
    }

    // ---------------------------------------------------------------- rigidity / elasticity

    /**
     * 0..1: how strongly the segment resists being stretched or compressed. 1 (default) = fixed length,
     * lower = a softer spring (see SoftConstraint for the frequencies).
     */
    public void setRigidity(float rigidity) {
        spring.setRigidity(rigidity);
    }

    public float getRigidity() {
        return spring.getRigidity();
    }

    /**
     * 0..1: how much of the stored energy comes back as motion. 1 = bouncy, rings for a long time;
     * 0 = returns to its length slowly without overshooting. Has no effect on a rigid segment.
     */
    public void setElasticity(float elasticity) {
        spring.setElasticity(elasticity);
    }

    public float getElasticity() {
        return spring.getElasticity();
    }

    public boolean isRigid() {
        return spring.isRigid();
    }

    /** Spring frequency in Hz (only meaningful when not rigid). */
    public float getFrequency() {
        return spring.frequency();
    }

    /**
     * Spring constant (force per metre of stretch) for the two end bodies as they are now, so that
     * force = stiffness x stretch. Infinite when rigid.
     */
    public float getStiffness() {
        float inverseMassSum = endA.getInverseMass() + endB.getInverseMass();
        if (inverseMassSum == 0) return Float.POSITIVE_INFINITY;
        return spring.stiffness(1f / inverseMassSum);
    }

    /** Current length minus rest length: positive = stretched, negative = compressed. */
    public float getStretch() {
        return distanceBetween(endA, endB) - length;
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