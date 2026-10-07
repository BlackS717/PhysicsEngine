package simulation;

import math.Vector2;
import physics.Body;
import physics.Collision;

import java.util.*;

public class Simulation {
    private final int fps;

    private final Vector2 gravitationalAcceleration = new Vector2(0, -9.81f);

    private final List<Body> bodies = new ArrayList<>();

    public Simulation(int fps){
        this.fps = fps;
    }

    private void updateBody(Body body, float dt){
        if(body == null) return;

        Vector2 gravitationalForce = getGravitationalForce(body);

        body.applyForces(gravitationalForce);

        body.integrate(dt);
    }

    private Vector2 getGravitationalForce(Body body){
        if(body == null) throw new NullPointerException();

        return gravitationalAcceleration.mult(body.getMass());
    }

    public void step(){
        float dt = 1.0f / fps;

        // Update all body
        for(int i = 0; i < bodies.size(); i ++){
            Body current = bodies.get(i);
            updateBody(current, dt);
        }

        List<Collision> collisionPairs= new ArrayList<>();
        // store collisions
        for(int i = 0; i < bodies.size(); i++){
            Body current = bodies.get(i);
            for(int j = i+1; j < bodies.size(); j++){
                Body other = bodies.get(j);

                if(current.isColliding(other)){
                    collisionPairs.add(new Collision(current, other));
                }

            }
        }

        // resolve collisions
        for(Collision collision: collisionPairs){
            collision.resolveCollision();
        }

    }

    public void reset(){
        for(Body body: bodies){
            body.reset();
        }
    }

    public int getFps(){
        return this.fps;
    }

    public List<Body> getBodies(){
        return Collections.unmodifiableList(this.bodies);
    }

    public void addBody(Body body){
        this.bodies.add(body);
    }

    public void removeBody(Body body){
        this.bodies.remove(body);
    }
}
