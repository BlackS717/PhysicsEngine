package physics;

import math.Vector2;

public class Collision {
    private final Body b1;
    private final Body b2;
    private Vector2 normal;
    private float penetration;

    public Collision(Body b1, Body b2){
        this.b1 = b1;
        this.b2 = b2;

        Vector2 b1Pos = b1.getTransform().getPosition();
        Vector2 b2Pos = b2.getTransform().getPosition();

        Vector2 p = b2Pos.sub(b1Pos);
        float distance = p.magnitude();

        calculateNormal(p, distance);
        calculatePenetration(distance);
    }

    public void resolveCollision(){
        correctPositions();
        resolveVelocities();
    }

    private void correctPositions(){
        float totalMass = b1.getMass() + b2.getMass();
        float b1Correction = (b2.getMass() / totalMass) * penetration;
        float b2Correction = (b1.getMass() / totalMass) * penetration;

        Vector2 b1Pos = b1.getTransform().getPosition();
        Vector2 b2Pos = b2.getTransform().getPosition();

        b1.getTransform().setPosition(b1Pos.sub(normal.mult(b1Correction)));
        b2.getTransform().setPosition(b2Pos.add(normal.mult(b2Correction)));
    }

    private void resolveVelocities(){
        Vector2 relativeVelocity = b2.getVelocity().sub(b1.getVelocity());
        float velocityAlongNormal = relativeVelocity.dotProduct(normal);

        if(velocityAlongNormal > 0){
            return;
        }

        float restitution = Math.max(b1.getRestitution(), b2.getRestitution());

        float impulseMagnitude = -(1 + restitution) * velocityAlongNormal;
        impulseMagnitude /= (1 / b1.getMass()) + (1 / b2.getMass());

        Vector2 impulse = normal.mult(impulseMagnitude);

        b1.setVelocity(b1.getVelocity().sub(impulse.div(b1.getMass())));
        b2.setVelocity(b2.getVelocity().add(impulse.div(b2.getMass())));
    }

    private void calculateNormal(Vector2 p, float distance){
        this.normal = p.div(distance);
    }

    private void calculatePenetration(float distance){
        float b1Radius = b1.getRenderInfo().getRadius();
        float b2Radius = b2.getRenderInfo().getRadius();

        this.penetration = (b1Radius + b2Radius) - distance;
    }

    public Body getB1() {
        return b1;
    }

    public Body getB2() {
        return b2;
    }

    public Vector2 getNormal() {
        return normal;
    }

    public float getPenetration() {
        return penetration;
    }
}
