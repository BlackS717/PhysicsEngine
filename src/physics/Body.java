package physics;

import math.Transform;
import math.Vector2;
import rendering.BodyRenderInfo;

import java.util.Enumeration;
import java.util.Set;

public class Body {
    // bodies are circular shape

    private Transform transform;
    private Vector2 velocity;
    private Vector2 acceleration;

    private final Transform initialTransform;
    private final Vector2 initialVelocity;
    private final Vector2 initialAcceleration;

    private float mass;

    private float restitionCoefficient = 1.0f; // 1.0f is perfectly elastic, 0.0f is perfectly inelastic

    // rendering
    private BodyRenderInfo renderInfo= new BodyRenderInfo();

    public Body(){
        this(new Transform(), 1.0f);
    }

    public Body(Transform transform, float mass) throws IllegalArgumentException{
        if(mass <= 0){
            throw new IllegalArgumentException("Mass must be positive.");
        }

        this.transform = transform;
        this.velocity = new Vector2(0,0);
        this.acceleration = new Vector2(0,0);
        this.mass = mass;

        initialTransform = new Transform(transform);
        initialVelocity = new Vector2(velocity);
        initialAcceleration = new Vector2(acceleration);
    }

    /***
     * dt is the time between each tick, 1 at 1fps and 0.5 at 2fps.
     * to get the value, you divide 1 by the desired fps.
     * */

    public void integrate(float dt) {
        updatePosition(dt);

        updateVelocity(dt);
    }

    private void updatePosition(float dt){
        this.transform.setPosition(
                this.transform.getPosition().add(
                        this.velocity.mult(dt).add(
                                this.acceleration.mult(dt*dt*0.5f)
                        )
                )
        );
    }

    private void updateVelocity(float dt){
        setVelocity(this.velocity.add(this.acceleration.mult(dt)));
    }

    public void applyForces(Vector2... forces){
        // reset the acceleration
        resetAcceleration();

        // apply the forces
        Vector2 forcesSum = new Vector2(0,0);

        for(Vector2 force: forces){
            forcesSum = forcesSum.add(force);
        }

        // update acceleration
        setAcceleration(forcesSum.div(mass));
    }

    private void resetAcceleration(){
        this.acceleration.setCoordinates(0,0);
    }

    // Getters and Setters
    public void setVelocity(Vector2 velocity){
        this.velocity = velocity;
    }
    public Vector2 getVelocity(){
        return this.velocity;
    }

    public void setAcceleration(Vector2 acceleration){
        this.acceleration = acceleration;
    }
    public Vector2 getAcceleration(){
        return this.acceleration;
    }

    public float getMass(){
        return this.mass;
    }

    public void setMass(float mass){
        this.mass = mass;
    }

    public void setRestitution(float restitution){
        this.restitionCoefficient = restitution;
    }

    public float getRestitution(){
        return this.restitionCoefficient;
    }

    public Transform getTransform() {
        return transform;
    }

    public BodyRenderInfo getRenderInfo(){
        return this.renderInfo;
    }

    public void reset(){
        this.transform = new Transform(initialTransform);
        this.acceleration = new Vector2(initialAcceleration);
        this.velocity = new Vector2(initialVelocity);
    }

    public boolean isColliding(Body other){
        Vector2 centerA = this.transform.getPosition();
        Vector2 centerB = other.transform.getPosition();

        Vector2 d = centerB.sub(centerA);

        float radiusA = renderInfo.getRadius() * this.transform.getScale().getX();
        float radiusB = other.renderInfo.getRadius() * other.transform.getScale().getX();

        return d.getX() * d.getX() + d.getY() * d.getY() <= (radiusA + radiusB) * (radiusA + radiusB);
    }



}
