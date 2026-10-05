package physics;

import math.Transform;
import math.Vector2;

public class Sphere {
    private Transform transform;
    private Vector2 velocity;
    private Vector2 acceleration;

    public Sphere(){
        this(new Transform());
    }

    public Sphere(Transform transform) {
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
