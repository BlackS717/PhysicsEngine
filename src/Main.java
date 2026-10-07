import math.Transform;
import math.Vector2;
import physics.Body;
import physics.BoxCollider;
import physics.CircleCollider;
import simulation.Simulation;
import rendering.Renderer;

import java.awt.*;

//TIP To <b>Run</b> code, press <shortcut actionId="Run"/> or
// click the <icon src="AllIcons.Actions.Execute"/> icon in the gutter.
void main() {
    Simulation sim = new Simulation(60);

    // Add static bodies to the simulation to create a box for the light to bounce around in
    Body floor = new Body(new Transform(new Vector2(0,0)), 5f);
    floor.setCollider(new BoxCollider(floor, 30.0f, 1.0f));
    floor.setRestitution(0f);
    floor.setStaticFriction(0.6f);
    floor.setDynamicFriction(0.4f);
    floor.getRenderInfo().setColor(Color.GRAY);
    floor.setStatic(true);

    Body rightWall = new Body(new Transform(new Vector2(15,15)), 5f);
    rightWall.setCollider(new BoxCollider(rightWall, 1.0f, 30.0f));
    rightWall.setRestitution(1f);
    rightWall.getRenderInfo().setColor(Color.GRAY);
    rightWall.setStatic(true);

    Body leftWall = new Body(new Transform(new Vector2(-15,15)), 5f);
    leftWall.setCollider(new BoxCollider(leftWall, 1.0f, 30.0f));
    leftWall.setRestitution(1f);
    leftWall.getRenderInfo().setColor(Color.GRAY);
    leftWall.setStatic(true);

    Body ceiling = new Body(new Transform(new Vector2(0,25)), 5f);
    ceiling.setCollider(new BoxCollider(ceiling, 30.0f, 1.0f));
    ceiling.setRestitution(0.5f);
    ceiling.getRenderInfo().setColor(Color.GRAY);
    ceiling.setStatic(true);

    sim.addBody(floor);
    sim.addBody(rightWall);
    sim.addBody(leftWall);
    sim.addBody(ceiling);

    // ########################################

    Body light = new Body(new Transform(new Vector2(-10,15)), 5.0f);
    light.setCollider(new CircleCollider(light, 1.0f));
//    light.setCollider(new BoxCollider(light, 1.0f, 1.0f));
    light.getRenderInfo().setColor(Color.BLUE);
    light.setRestitution(0f);
    light.setStaticFriction(0.6f);
    light.setDynamicFriction(0.4f);
    light.setVelocity(new Vector2(4f, 0f));

    sim.addBody(light);

    Body stick = new Body(new Transform(new Vector2(-10,10)), 5.0f);
    stick.setCollider(new BoxCollider(stick, 5.0f, 1.0f));
    stick.getRenderInfo().setColor(Color.RED);
    stick.setRestitution(0f);
    stick.setStaticFriction(0.6f);
    stick.setDynamicFriction(0.4f);

    stick.setVelocity(new Vector2(0f, 15f));

    sim.addBody(stick);

    Renderer.run(() -> sim);
}
