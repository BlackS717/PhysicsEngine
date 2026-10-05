package physics;

import math.Transform;
import math.Vector2;

public class Body {
    private Transform transform;
    private Vector2 velocity;
    private Vector2 acceleration;

    private float mass;

    public Body(){
        this(new Transform());
    }

    public Body(Transform transform) {
        this.transform = transform;
        this.velocity = new Vector2(0,0);
        this.acceleration = new Vector2(0,0);
    }

    /***
     * dt is the time between each tick, 1 at 1fps and 0.5 at 2fps.
     * to get the value, you divide 1 by the desired fps.
     * */

    private void integrate(float dt) {
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

    public Transform getTransform() {
        return transform;
    }

}
