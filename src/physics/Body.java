package physics;

import math.Transform;
import math.Vector2;
import rendering.BodyRenderInfo;

public class Body {
    private Transform transform;
    private Vector2 velocity;
    private Vector2 acceleration;

    private float mass;

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

    public void setAcceleration(Vector2 acceleration){
        this.acceleration = acceleration;
    }

    public float getMass(){
        return this.mass;
    }

    public void setMass(float mass){
        this.mass = mass;
    }

    public Transform getTransform() {
        return transform;
    }

    public BodyRenderInfo getRenderInfo(){
        return this.renderInfo;
    }

}
