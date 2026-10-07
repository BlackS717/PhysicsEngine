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
    floor.setStaticFriction(0.5f);
    floor.setDynamicFriction(0.2f);
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

    Color[] colors = {Color.RED, Color.GREEN, Color.BLUE, Color.YELLOW, Color.CYAN};

    int numObjects = 10;

    // test multiple box stacking
    for(int i = 0; i < numObjects; i++) {
        // randomize position and size of the box
        Random rand = new Random();
        Vector2 position = new Vector2(rand.nextFloat() * 10 - 5, 1.5f + i * 2.5f);
        Body box = new Body(new Transform( position), 1f);

//        box.setCollider(new BoxCollider(box, 1f, 1f));
        box.setCollider(new CircleCollider(box, 0.5f));
        box.setRestitution(0.75f);
        box.setStaticFriction(0.6f);
        box.setDynamicFriction(0.4f);
        box.getRenderInfo().setColor(colors[i % colors.length]);

        sim.addBody(box);
    }

    Renderer.run(() -> sim);
}
