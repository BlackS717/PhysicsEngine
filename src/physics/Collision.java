package physics;

import math.Vector2;

import java.util.List;

/**
 * A contact between two colliders. The normal points from c1 to c2.
 * A collision can have several contact points (e.g. 2 for a box lying flat on another box).
 *
 * Each collider's body is wrapped in a CollisionSide, so the same solver handles ordinary bodies and the middle
 * box of a Segment (where an impulse is shared between the segment's two end bodies).
 *
 * Velocities are solved with sequential impulses using ACCUMULATED impulses: over the solver iterations of one
 * step, every contact remembers the total impulse applied so far and may reduce it again (never below zero for
 * the normal impulse). That is what lets stacks settle instead of pumping energy into each other.
 *
 * Per step: prepare() once, resolveVelocities() several times, settleBodies() once, correctPositions() once.
 */
public class Collision {
    // positional correction tuning: fix 80% of the overlap, ignore tiny overlaps to avoid jitter
    private static final float CORRECTION_PERCENT = 0.8f;
    private static final float SLOP = 0.01f;

    // impacts slower than this (m/s) don't bounce. Without it, the tiny speed gravity adds each step
    // (g * dt ~ 0.16 m/s) is reflected back as a bounce, so resting bodies jitter forever.
    private static final float RESTITUTION_VELOCITY_THRESHOLD = 0.7f;

    private static final int CONVENIENCE_ITERATIONS = 10;

    private final Collider c1;
    private final Collider c2;
    private final CollisionSide side1;
    private final CollisionSide side2;
    private final Vector2 normal;
    private final Vector2 tangent; // perpendicular to the normal, fixed for the whole step
    private final float penetration;
    private final List<Vector2> contactPoints; // world space, at least one

    // ---- per-contact solver data, filled in by prepare()
    private float[] normalMass;        // 1 / (effective inverse mass along the normal)
    private float[] tangentMass;       // same along the tangent
    private float[] bounce;            // target normal velocity after the impact (restitution)
    private float[] normalImpulseSum;  // accumulated normal impulse
    private float[] tangentImpulseSum; // accumulated friction impulse
    private float muStatic;
    private float muDynamic;

    public Collision(Collider c1, Collider c2, Vector2 normal, float penetration, Vector2 contactPoint) {
        this(c1, c2, normal, penetration, List.of(contactPoint));
    }

    public Collision(Collider c1, Collider c2, Vector2 normal, float penetration, List<Vector2> contactPoints) {
        if (contactPoints == null || contactPoints.isEmpty()) {
            throw new IllegalArgumentException("A collision needs at least one contact point.");
        }

        this.c1 = c1;
        this.c2 = c2;
        this.side1 = CollisionSide.of(c1.getBody());
        this.side2 = CollisionSide.of(c2.getBody());
        this.normal = normal;
        this.tangent = new Vector2(-normal.getY(), normal.getX());
        this.penetration = penetration;
        this.contactPoints = List.copyOf(contactPoints);
    }

    /** Convenience: the whole resolution for this collision alone (Simulation uses the separate steps). */
    public void resolveCollision() {
        prepare();
        for (int i = 0; i < CONVENIENCE_ITERATIONS; i++) {
            resolveVelocities();
        }
        settleBodies();
        correctPositions();
    }

    /** True when neither side can move, so there is nothing to resolve. */
    private boolean isBetweenImmovableBodies() {
        return side1.inverseMass() + side2.inverseMass() == 0;
    }

    // ---------------------------------------------------------------- velocities

    /** Call once per step, before the solver iterations: computes masses and restitution targets. */
    public void prepare() {
        int n = contactPoints.size();

        normalMass = new float[n];
        tangentMass = new float[n];
        bounce = new float[n];
        normalImpulseSum = new float[n];
        tangentImpulseSum = new float[n];

        muStatic = (float) Math.sqrt(side1.staticFriction() * side2.staticFriction());
        muDynamic = (float) Math.sqrt(side1.dynamicFriction() * side2.dynamicFriction());

        float restitution = Math.max(side1.restitution(), side2.restitution());

        for (int i = 0; i < n; i++) {
            Vector2 point = contactPoints.get(i);

            float normalDenominator = side1.inverseMassAlong(point, normal) + side2.inverseMassAlong(point, normal);
            normalMass[i] = normalDenominator > 0 ? 1f / normalDenominator : 0f;

            float tangentDenominator = side1.inverseMassAlong(point, tangent) + side2.inverseMassAlong(point, tangent);
            tangentMass[i] = tangentDenominator > 0 ? 1f / tangentDenominator : 0f;

            // restitution target, from the velocity at the moment of impact (slow impacts don't bounce)
            float approachSpeed = -relativeVelocityAt(point).dotProduct(normal);
            bounce[i] = approachSpeed > RESTITUTION_VELOCITY_THRESHOLD ? restitution * approachSpeed : 0f;
        }
    }

    /** One solver pass over every contact point of this collision. Call prepare() first. */
    public void resolveVelocities() {
        if (isBetweenImmovableBodies()) return;

        for (int i = 0; i < contactPoints.size(); i++) {
            solveFriction(i);
            solveNormal(i);
        }
    }

    private void solveNormal(int i) {
        Vector2 point = contactPoints.get(i);
        float velocityAlongNormal = relativeVelocityAt(point).dotProduct(normal);

        // impulse that would bring the normal velocity to the target (0, or the bounce speed)
        float impulse = normalMass[i] * (bounce[i] - velocityAlongNormal);

        // accumulated clamp: the total normal impulse can shrink again, but never becomes a pull
        float newSum = Math.max(normalImpulseSum[i] + impulse, 0f);
        impulse = newSum - normalImpulseSum[i];
        normalImpulseSum[i] = newSum;

        applyImpulse(normal.mult(impulse), point);
    }

    private void solveFriction(int i) {
        Vector2 point = contactPoints.get(i);
        float velocityAlongTangent = relativeVelocityAt(point).dotProduct(tangent);

        float impulse = -velocityAlongTangent * tangentMass[i];

        // Coulomb's law with separate static and dynamic coefficients, limited by the normal force at this contact
        float newSum = tangentImpulseSum[i] + impulse;
        if (Math.abs(newSum) > muStatic * normalImpulseSum[i]) {
            // static limit exceeded -> the contact slips, so only dynamic friction acts
            newSum = Math.copySign(muDynamic * normalImpulseSum[i], newSum);
        }
        // otherwise the contact grips and the full impulse is applied

        impulse = newSum - tangentImpulseSum[i];
        tangentImpulseSum[i] = newSum;

        applyImpulse(tangent.mult(impulse), point);
    }

    /**
     * Snaps barely-moving bodies to rest. Call once after all solver iterations.
     * Only happens if this contact actually pressed the bodies together, so a body leaving a surface is not frozen.
     */
    public void settleBodies() {
        if (isBetweenImmovableBodies() || normalImpulseSum == null) return;

        boolean touching = false;
        for (float sum : normalImpulseSum) {
            if (sum > 0) touching = true;
        }

        if (touching) {
            if (isSupportedAtRest(c1)) side1.settleIfSlow();
            if (isSupportedAtRest(c2)) side2.settleIfSlow();
        }
    }

    /**
     * Slow movement only means "at rest" if the contact can actually hold the body up.
     * A circle is always supported by a single contact (it rolls). A box touching with a single point is
     * balancing on a corner and is about to tip over, so it must keep moving; a flat contact has 2+ points.
     */
    private boolean isSupportedAtRest(Collider collider) {
        return collider instanceof CircleCollider || contactPoints.size() >= 2;
    }

    // ---------------------------------------------------------------- positions

    public void correctPositions() {
        if (isBetweenImmovableBodies()) return;

        // push apart at the middle of the contact points
        Vector2 sum = new Vector2(0, 0);
        for (Vector2 point : contactPoints) {
            sum = sum.add(point);
        }
        Vector2 point = sum.div(contactPoints.size());

        float totalInverseMass = side1.positionalInverseMass(point) + side2.positionalInverseMass(point);
        if (totalInverseMass == 0) return;

        float depth = Math.max(penetration - SLOP, 0f);
        Vector2 displacement = normal.mult(depth * CORRECTION_PERCENT / totalInverseMass);

        side1.shiftPosition(displacement.mult(-1), point);
        side2.shiftPosition(displacement, point);
    }

    // ---------------------------------------------------------------- helpers

    /** Velocity of side 2's contact point relative to side 1's contact point, including spin. */
    private Vector2 relativeVelocityAt(Vector2 point) {
        return side2.velocityAt(point).sub(side1.velocityAt(point));
    }

    /** Applies +impulse to side 2 and -impulse to side 1 at the contact point. */
    private void applyImpulse(Vector2 impulse, Vector2 point) {
        side1.applyImpulse(impulse.mult(-1), point);
        side2.applyImpulse(impulse, point);
    }

    // ---------------------------------------------------------------- getters

    public Vector2 getNormal() {
        return normal;
    }

    public float getPenetration() {
        return penetration;
    }

    public List<Vector2> getContactPoints() {
        return contactPoints;
    }
}