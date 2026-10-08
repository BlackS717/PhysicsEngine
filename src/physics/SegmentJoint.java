package physics;

import math.Vector2;

/**
 * Keeps the angle between two linked segments (two segments that share an end body), so a structure made of
 * segments can resist bending and spring back to its original shape.
 *
 *            A                 A, B and C are bodies; B is the body the two segments share.
 *             \                The joint controls the angle at B, measured from B->A to B->C.
 *              \
 *               B ------ C
 *
 * It has the same two values as a Segment (see SoftConstraint):
 * - rigidity 1 (the default) keeps the angle fixed (a welded joint). Lower it for an angular spring.
 * - elasticity says how much of the stored energy comes back as motion (bouncy) instead of being damped.
 *
 * The rest angle is the angle the segments have when the joint is created, unless you give one.
 * Add it with Simulation.addJoint(...) after the two segments have been added.
 */
public class SegmentJoint {
    private static final float MIN_ARM_LENGTH = 1e-4f;
    private static final float MAX_POSITION_CORRECTION = 0.2f; // radians per sweep, keeps rigid joints from jumping

    private final Segment first;
    private final Segment second;
    private final Body shared; // B
    private final Body endA;   // other end of the first segment
    private final Body endC;   // other end of the second segment
    private final float restAngle;

    private final SoftConstraint spring = new SoftConstraint();

    // per-step data, filled in by prepare()
    private boolean active;
    private Vector2 gradientA;
    private Vector2 gradientB;
    private Vector2 gradientC;
    private float inverseMass;

    /** The rest angle is the angle between the two segments right now. */
    public SegmentJoint(Segment first, Segment second) {
        this(first, second, Float.NaN);
    }

    /** @param restAngle radians, counter-clockwise from the first segment's arm (B->A) to the second's (B->C) */
    public SegmentJoint(Segment first, Segment second, float restAngle) {
        if (first == null || second == null || first == second) {
            throw new IllegalArgumentException("A joint needs two different segments.");
        }

        Body[] ends = findSharedBody(first, second);
        this.first = first;
        this.second = second;
        this.shared = ends[0];
        this.endA = ends[1];
        this.endC = ends[2];

        this.restAngle = Float.isNaN(restAngle) ? getAngle() : restAngle;
    }

    /** @return {shared body, other end of first, other end of second} */
    private static Body[] findSharedBody(Segment first, Segment second) {
        Body a1 = first.getEndA(), b1 = first.getEndB();
        Body a2 = second.getEndA(), b2 = second.getEndB();

        boolean sharesA = (a1 == a2 || a1 == b2);
        boolean sharesB = (b1 == a2 || b1 == b2);

        if (sharesA == sharesB) {
            throw new IllegalArgumentException(
                    sharesA ? "The two segments share both of their bodies." : "The two segments don't share a body.");
        }

        Body shared = sharesA ? a1 : b1;
        Body otherOfFirst = sharesA ? b1 : a1;
        Body otherOfSecond = (a2 == shared) ? b2 : a2;

        return new Body[]{shared, otherOfFirst, otherOfSecond};
    }

    // ---------------------------------------------------------------- geometry

    /** Current angle in radians, from B->A to B->C, counter-clockwise, in (-PI, PI]. */
    public float getAngle() {
        Vector2 u = endA.getTransform().getPosition().sub(shared.getTransform().getPosition());
        Vector2 v = endC.getTransform().getPosition().sub(shared.getTransform().getPosition());
        return angleBetween(u, v);
    }

    /** Current angle minus rest angle, wrapped to (-PI, PI]: positive = opened up, negative = closed. */
    public float getBend() {
        return wrap(getAngle() - restAngle);
    }

    public float getRestAngle() {
        return restAngle;
    }

    private static float angleBetween(Vector2 u, Vector2 v) {
        float cross = u.getX() * v.getY() - u.getY() * v.getX();
        return (float) Math.atan2(cross, u.dotProduct(v));
    }

    private static float wrap(float angle) {
        while (angle > Math.PI) angle -= (float) (2 * Math.PI);
        while (angle <= -Math.PI) angle += (float) (2 * Math.PI);
        return angle;
    }

    /**
     * Fills in the gradient of the angle with respect to the three body positions (how each body would have to
     * move to change the angle) and the constraint's inverse mass. Returns false if it can't be computed.
     */
    private boolean computeGradients() {
        Vector2 b = shared.getTransform().getPosition();
        Vector2 u = endA.getTransform().getPosition().sub(b);
        Vector2 v = endC.getTransform().getPosition().sub(b);

        float uu = u.dotProduct(u);
        float vv = v.dotProduct(v);
        if (uu < MIN_ARM_LENGTH * MIN_ARM_LENGTH || vv < MIN_ARM_LENGTH * MIN_ARM_LENGTH) return false;

        // angle = atan2(v) - atan2(u), so the gradient of each arm is perpendicular to it, divided by its length
        gradientA = new Vector2(u.getY() / uu, -u.getX() / uu);
        gradientC = new Vector2(-v.getY() / vv, v.getX() / vv);
        gradientB = gradientA.add(gradientC).mult(-1); // moving all three together changes nothing

        inverseMass = endA.getInverseMass() * gradientA.dotProduct(gradientA)
                + shared.getInverseMass() * gradientB.dotProduct(gradientB)
                + endC.getInverseMass() * gradientC.dotProduct(gradientC);

        return inverseMass > 0f;
    }

    // ---------------------------------------------------------------- constraint (solved by Simulation)

    /** Call once per step before the solver iterations. */
    public void prepare(float dt) {
        active = computeGradients();
        if (!active) return;

        spring.prepare(1f / inverseMass, getBend(), dt);
    }

    /** One solver iteration on how fast the angle is changing. */
    public void solveVelocityConstraint() {
        if (!active) return;

        float angularSpeed = gradientA.dotProduct(endA.getVelocity())
                + gradientB.dotProduct(shared.getVelocity())
                + gradientC.dotProduct(endC.getVelocity());

        float impulse = spring.solve(angularSpeed, inverseMass);

        endA.setVelocity(endA.getVelocity().add(gradientA.mult(impulse * endA.getInverseMass())));
        shared.setVelocity(shared.getVelocity().add(gradientB.mult(impulse * shared.getInverseMass())));
        endC.setVelocity(endC.getVelocity().add(gradientC.mult(impulse * endC.getInverseMass())));
    }

    /** Rotates the arms back to the rest angle. Only for rigid joints: a soft joint's spring pulls it back. */
    public void solvePositionConstraint() {
        if (!spring.isRigid()) return;
        if (!computeGradients()) return;

        float error = Math.max(-MAX_POSITION_CORRECTION, Math.min(MAX_POSITION_CORRECTION, getBend()));
        float impulse = -error / inverseMass;

        move(endA, gradientA, impulse);
        move(shared, gradientB, impulse);
        move(endC, gradientC, impulse);
    }

    private static void move(Body body, Vector2 gradient, float impulse) {
        body.getTransform().setPosition(body.getTransform().getPosition()
                .add(gradient.mult(impulse * body.getInverseMass())));
    }

    // ---------------------------------------------------------------- rigidity / elasticity

    /** 0..1: how strongly the angle is held. 1 (default) = fixed angle, lower = a softer angular spring. */
    public void setRigidity(float rigidity) {
        spring.setRigidity(rigidity);
    }

    public float getRigidity() {
        return spring.getRigidity();
    }

    /** 0..1: how much of the stored energy comes back as motion (1 = bouncy, 0 = slow, no overshoot). */
    public void setElasticity(float elasticity) {
        spring.setElasticity(elasticity);
    }

    public float getElasticity() {
        return spring.getElasticity();
    }

    public boolean isRigid() {
        return spring.isRigid();
    }

    // ---------------------------------------------------------------- getters

    public Segment getFirst() {
        return first;
    }

    public Segment getSecond() {
        return second;
    }

    /** The body the two segments share (the corner of the angle). */
    public Body getSharedBody() {
        return shared;
    }
}