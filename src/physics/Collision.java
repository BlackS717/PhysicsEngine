package physics;

import math.Vector2;

import java.util.List;

/**
 * A contact between two colliders. The normal points from c1 (b1) to c2 (b2).
 * A collision can have several contact points (e.g. 2 for a box lying flat on another box).
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
    private final Body b1;
    private final Body b2;
    private final Vector2 normal;
    private final Vector2 tangent; // perpendicular to the normal, fixed for the whole step
    private final float penetration;
    private final List<Vector2> contactPoints; // world space, at least one

    // ---- per-contact solver data, filled in by prepare()
    private Vector2[] r1;              // lever arm: b1 centre -> contact
    private Vector2[] r2;              // lever arm: b2 centre -> contact
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
        this.b1 = c1.getBody();
        this.b2 = c2.getBody();
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

    /** True when neither body can move, so there is nothing to resolve. */
    private boolean isBetweenImmovableBodies() {
        return b1.getInverseMass() + b2.getInverseMass() == 0;
    }

    // ---------------------------------------------------------------- velocities

    /** Call once per step, before the solver iterations: computes lever arms, masses and restitution targets. */
    public void prepare() {
        int n = contactPoints.size();

        r1 = new Vector2[n];
        r2 = new Vector2[n];
        normalMass = new float[n];
        tangentMass = new float[n];
        bounce = new float[n];
        normalImpulseSum = new float[n];
        tangentImpulseSum = new float[n];

        muStatic = (float) Math.sqrt(b1.getStaticFriction() * b2.getStaticFriction());
        muDynamic = (float) Math.sqrt(b1.getDynamicFriction() * b2.getDynamicFriction());

        float restitution = Math.max(b1.getRestitution(), b2.getRestitution());

        float invMass1 = b1.getInverseMass();
        float invMass2 = b2.getInverseMass();
        float invInertia1 = b1.getInverseInertia();
        float invInertia2 = b2.getInverseInertia();

        for (int i = 0; i < n; i++) {
            Vector2 contact = contactPoints.get(i);
            r1[i] = contact.sub(b1.getTransform().getPosition());
            r2[i] = contact.sub(b2.getTransform().getPosition());

            float r1n = cross(r1[i], normal);
            float r2n = cross(r2[i], normal);
            float normalDenominator = invMass1 + invMass2 + r1n * r1n * invInertia1 + r2n * r2n * invInertia2;
            normalMass[i] = normalDenominator > 0 ? 1f / normalDenominator : 0f;

            float r1t = cross(r1[i], tangent);
            float r2t = cross(r2[i], tangent);
            float tangentDenominator = invMass1 + invMass2 + r1t * r1t * invInertia1 + r2t * r2t * invInertia2;
            tangentMass[i] = tangentDenominator > 0 ? 1f / tangentDenominator : 0f;

            // restitution target, from the velocity at the moment of impact (slow impacts don't bounce)
            float approachSpeed = -relativeVelocityAtContact(r1[i], r2[i]).dotProduct(normal);
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
        float velocityAlongNormal = relativeVelocityAtContact(r1[i], r2[i]).dotProduct(normal);

        // impulse that would bring the normal velocity to the target (0, or the bounce speed)
        float impulse = normalMass[i] * (bounce[i] - velocityAlongNormal);

        // accumulated clamp: the total normal impulse can shrink again, but never becomes a pull
        float newSum = Math.max(normalImpulseSum[i] + impulse, 0f);
        impulse = newSum - normalImpulseSum[i];
        normalImpulseSum[i] = newSum;

        applyImpulse(normal.mult(impulse), r1[i], r2[i]);
    }

    private void solveFriction(int i) {
        float velocityAlongTangent = relativeVelocityAtContact(r1[i], r2[i]).dotProduct(tangent);

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

        applyImpulse(tangent.mult(impulse), r1[i], r2[i]);
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
            if (isSupportedAtRest(c1)) b1.settleIfSlow();
            if (isSupportedAtRest(c2)) b2.settleIfSlow();
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

        float invMass1 = b1.getInverseMass();
        float invMass2 = b2.getInverseMass();
        float totalInverseMass = invMass1 + invMass2;

        float depth = Math.max(penetration - SLOP, 0f);
        Vector2 correction = normal.mult(depth * CORRECTION_PERCENT / totalInverseMass);

        b1.getTransform().setPosition(b1.getTransform().getPosition().sub(correction.mult(invMass1)));
        b2.getTransform().setPosition(b2.getTransform().getPosition().add(correction.mult(invMass2)));
    }

    // ---------------------------------------------------------------- helpers

    /** Velocity of b2's contact point relative to b1's contact point, including spin. */
    private Vector2 relativeVelocityAtContact(Vector2 r1, Vector2 r2) {
        Vector2 v1 = b1.getVelocity().add(angularToLinear(b1.getAngularVelocity(), r1));
        Vector2 v2 = b2.getVelocity().add(angularToLinear(b2.getAngularVelocity(), r2));
        return v2.sub(v1);
    }

    /** Applies +impulse to b2 and -impulse to b1 at the contact point. */
    private void applyImpulse(Vector2 impulse, Vector2 r1, Vector2 r2) {
        b1.setVelocity(b1.getVelocity().sub(impulse.mult(b1.getInverseMass())));
        b1.setAngularVelocity(b1.getAngularVelocity() - b1.getInverseInertia() * cross(r1, impulse));

        b2.setVelocity(b2.getVelocity().add(impulse.mult(b2.getInverseMass())));
        b2.setAngularVelocity(b2.getAngularVelocity() + b2.getInverseInertia() * cross(r2, impulse));
    }

    /** 2D cross product (z component of a x b). */
    private static float cross(Vector2 a, Vector2 b) {
        return a.getX() * b.getY() - a.getY() * b.getX();
    }

    /** Linear velocity at offset r caused by spinning at omega: omega x r. */
    private static Vector2 angularToLinear(float omega, Vector2 r) {
        return new Vector2(-omega * r.getY(), omega * r.getX());
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