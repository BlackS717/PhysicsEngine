package simulation;

import math.Vector2;
import physics.Body;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Simulation {
    private int fps;

    private Vector2 gravitationalAcceleration = new Vector2(0, -9.81f);

    private List<Body> bodies = new ArrayList<>();

    public Simulation(int fps){
        this.fps = fps;
    }

    private void updateBody(Body body, float dt){
        if(body == null) return;

        body.applyForces(getGravitationalForce(body), new Vector2(5.0f, 0f));

        body.integrate(dt);
    }

    private Vector2 getGravitationalForce(Body body){
        if(body == null) throw new NullPointerException();

        return gravitationalAcceleration.mult(body.getMass());
    }

    public void step(){
        float dt = 1.0f / fps;

        // loop through all body and update them
        for(Body body: bodies){
            updateBody(body, dt);
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
