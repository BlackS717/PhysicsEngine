package physics;

import math.Vector2;

public class Collision {
    private final Collider c1;
    private final Collider c2;
    private final Body b1;
    private final Body b2;
    private final Vector2 normal;
    private final float penetration;

    public Collision(Collider c1, Collider c2, Vector2 normal, float penetration){
        this.c1 = c1;
        this.c2 = c2;
        this.b1 = c1.getBody();
        this.b2 = c2.getBody();
        this.normal = normal;
        this.penetration = penetration;
    }

    public void resolveCollision(){
        correctPositions();
        resolveVelocities();
    }

    private void correctPositions(){
        float totalInverseMass = (b1.getInverseMass()) + (b2.getInverseMass());
        Vector2 correction = normal.mult(penetration / totalInverseMass);

        Vector2 b1Correction = correction.mult(b1.getInverseMass());
        Vector2 b2Correction = correction.mult(b2.getInverseMass());

        b1.getTransform().setPosition(b1.getTransform().getPosition().sub(b1Correction));
        b2.getTransform().setPosition(b2.getTransform().getPosition().add(b2Correction));
    }

    private void resolveVelocities(){
        Vector2 relativeVelocity = b2.getVelocity().sub(b1.getVelocity());
        float velocityAlongNormal = relativeVelocity.dotProduct(normal);

        if(velocityAlongNormal > 0){
            return;
        }

        float restitution = Math.max(b1.getRestitution(), b2.getRestitution());

        float impulseMagnitude = -(1 + restitution) * velocityAlongNormal;
        impulseMagnitude /= (b1.getInverseMass()) + (b2.getInverseMass());

        Vector2 impulse = normal.mult(impulseMagnitude);

        b1.setVelocity(b1.getVelocity().sub(impulse.div(b1.getMass())));
        b2.setVelocity(b2.getVelocity().add(impulse.div(b2.getMass())));
    }

    public Vector2 getNormal() {
        return normal;
    }

    public float getPenetration() {
        return penetration;
    }
}
