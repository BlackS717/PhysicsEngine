package physics;

import math.Vector2;

/**
 * A contact between two colliders. The normal points from c1 (b1) to c2 (b2).
 */
public class Collision {
    // positional correction tuning: fix 80% of the overlap, ignore tiny overlaps to avoid jitter
    private static final float CORRECTION_PERCENT = 0.8f;
    private static final float SLOP = 0.01f;

    private final Collider c1;
    private final Collider c2;
    private final Body b1;
    private final Body b2;
    private final Vector2 normal;
    private final float penetration;

    public Collision(Collider c1, Collider c2, Vector2 normal, float penetration) {
        this.c1 = c1;
        this.c2 = c2;
        this.b1 = c1.getBody();
        this.b2 = c2.getBody();
        this.normal = normal;
        this.penetration = penetration;
    }

    public void resolveCollision() {
        // two immovable bodies: nothing to resolve (also avoids dividing by zero)
        if (b1.getInverseMass() + b2.getInverseMass() == 0) return;

        correctPositions();
        resolveVelocities();
    }

    private void correctPositions() {
        float invMass1 = b1.getInverseMass();
        float invMass2 = b2.getInverseMass();
        float totalInverseMass = invMass1 + invMass2;

        float depth = Math.max(penetration - SLOP, 0f);
        Vector2 correction = normal.mult(depth * CORRECTION_PERCENT / totalInverseMass);

        b1.getTransform().setPosition(b1.getTransform().getPosition().sub(correction.mult(invMass1)));
        b2.getTransform().setPosition(b2.getTransform().getPosition().add(correction.mult(invMass2)));
    }

    private void resolveVelocities() {
        float invMass1 = b1.getInverseMass();
        float invMass2 = b2.getInverseMass();

        Vector2 relativeVelocity = b2.getVelocity().sub(b1.getVelocity());
        float velocityAlongNormal = relativeVelocity.dotProduct(normal);

        // already separating
        if (velocityAlongNormal > 0) {
            return;
        }

        float restitution = Math.max(b1.getRestitution(), b2.getRestitution());

        float impulseMagnitude = -(1 + restitution) * velocityAlongNormal / (invMass1 + invMass2);
        Vector2 impulse = normal.mult(impulseMagnitude);

        b1.setVelocity(b1.getVelocity().sub(impulse.mult(invMass1)));
        b2.setVelocity(b2.getVelocity().add(impulse.mult(invMass2)));
    }

    public Vector2 getNormal() {
        return normal;
    }

    public float getPenetration() {
        return penetration;
    }
}