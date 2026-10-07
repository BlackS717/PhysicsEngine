package physics;

import math.Vector2;

import java.util.List;

/**
 * A contact between two colliders. The normal points from c1 (b1) to c2 (b2).
 * A collision can have several contact points (e.g. 2 for a box lying flat on another box).
 * Impulses are applied at each contact point, so they change both linear and angular velocity,
 * and a Coulomb friction impulse is what makes bodies spin / roll.
 *
 * Typical use per step: call resolveVelocities() a few times, then correctPositions() once.
 */
public class Collision {
    // positional correction tuning: fix 80% of the overlap, ignore tiny overlaps to avoid jitter
    private static final float CORRECTION_PERCENT = 0.8f;
    private static final float SLOP = 0.01f;

    // impacts slower than this (m/s) don't bounce. Without it, the tiny speed gravity adds each step
    // (g * dt ~ 0.16 m/s) is reflected back as a bounce, so resting bodies jitter forever.
    private static final float RESTITUTION_VELOCITY_THRESHOLD = 0.7f;

    private final Collider c1;
    private final Collider c2;
    private final Body b1;
    private final Body b2;
    private final Vector2 normal;
    private final float penetration;
    private final List<Vector2> contactPoints; // world space, at least one

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
        this.penetration = penetration;
        this.contactPoints = List.copyOf(contactPoints);
    }

    /** Convenience: one velocity pass followed by position correction. */
    public void resolveCollision() {
        resolveVelocities();
        correctPositions();
    }

    /** True when neither body can move, so there is nothing to resolve. */
    private boolean isBetweenImmovableBodies() {
        return b1.getInverseMass() + b2.getInverseMass() == 0;
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

    // ---------------------------------------------------------------- velocities

    public void resolveVelocities() {
        if (isBetweenImmovableBodies()) return;

        boolean impulseApplied = false;
        for (Vector2 contact : contactPoints) {
            impulseApplied |= resolveContact(contact);
        }

        // a body in resting contact that is barely moving is snapped to rest
        // (skipped when every contact was already separating, so a body leaving a surface is never frozen)
        if (impulseApplied) {
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

    /**
     * Resolves one contact point. Each impulse is divided by the number of contact points so that the total
     * impulse for the whole collision is about the same as it would be for a single contact.
     *
     * @return false if the bodies were already moving apart at this point (nothing applied)
     */
    private boolean resolveContact(Vector2 contact) {
        int contactCount = contactPoints.size();

        float invMass1 = b1.getInverseMass();
        float invMass2 = b2.getInverseMass();
        float invInertia1 = b1.getInverseInertia();
        float invInertia2 = b2.getInverseInertia();

        // lever arms from each body's centre to the contact point
        Vector2 r1 = contact.sub(b1.getTransform().getPosition());
        Vector2 r2 = contact.sub(b2.getTransform().getPosition());

        // ---- normal impulse (bounce)
        Vector2 relativeVelocity = relativeVelocityAtContact(r1, r2);
        float velocityAlongNormal = relativeVelocity.dotProduct(normal);

        // already separating at this point
        if (velocityAlongNormal > 0) {
            return false;
        }

        float r1n = cross(r1, normal);
        float r2n = cross(r2, normal);
        float normalDenominator = invMass1 + invMass2
                + r1n * r1n * invInertia1
                + r2n * r2n * invInertia2;

        float restitution = Math.max(b1.getRestitution(), b2.getRestitution());
        if (-velocityAlongNormal < RESTITUTION_VELOCITY_THRESHOLD) {
            restitution = 0f; // slow impact (e.g. resting contact): absorb it instead of bouncing
        }

        float normalImpulse = -(1 + restitution) * velocityAlongNormal / normalDenominator / contactCount;

        applyImpulse(normal.mult(normalImpulse), r1, r2);

        // ---- friction impulse (makes bodies spin / roll)
        relativeVelocity = relativeVelocityAtContact(r1, r2);

        Vector2 tangent = relativeVelocity.sub(normal.mult(relativeVelocity.dotProduct(normal)));
        float tangentLength = tangent.magnitude();

        if (tangentLength >= 1e-6f) { // otherwise there is no sliding at the contact
            tangent = tangent.div(tangentLength);

            float r1t = cross(r1, tangent);
            float r2t = cross(r2, tangent);
            float tangentDenominator = invMass1 + invMass2
                    + r1t * r1t * invInertia1
                    + r2t * r2t * invInertia2;

            float frictionImpulse = -relativeVelocity.dotProduct(tangent) / tangentDenominator / contactCount;

            // Coulomb's law with separate static and dynamic coefficients
            float muStatic = (float) Math.sqrt(b1.getStaticFriction() * b2.getStaticFriction());
            float muDynamic = (float) Math.sqrt(b1.getDynamicFriction() * b2.getDynamicFriction());

            if (Math.abs(frictionImpulse) > normalImpulse * muStatic) {
                // static limit exceeded -> the contact slips, so only dynamic friction acts
                frictionImpulse = Math.copySign(normalImpulse * muDynamic, frictionImpulse);
            }
            // otherwise the impulse needed to stop the sliding is within the static limit: the contact grips

            applyImpulse(tangent.mult(frictionImpulse), r1, r2);
        }

        return true;
    }

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