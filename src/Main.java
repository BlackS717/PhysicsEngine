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
    Renderer.run(() -> {
        Simulation sim = new Simulation(480);

        // Add static bodies to the simulation to create a box for the light to bounce around in
        Body floor = new Body(new Transform(new Vector2(0,0)), 5f);
        floor.setCollider(new BoxCollider(floor, 50, 1.0f));
        floor.setRestitution(0f);
        floor.setStaticFriction(0.5f);
        floor.setDynamicFriction(0.2f);
        floor.getRenderInfo().setColor(Color.GRAY);
        floor.setStatic(true);

        Body rightWall = new Body(new Transform(new Vector2(25,25)), 5f);
        rightWall.setCollider(new BoxCollider(rightWall, 1.0f, 50));
        rightWall.setRestitution(1f);
        rightWall.getRenderInfo().setColor(Color.GRAY);
        rightWall.setStatic(true);

        Body leftWall = new Body(new Transform(new Vector2(-25,25)), 5f);
        leftWall.setCollider(new BoxCollider(leftWall, 1.0f, 50));
        leftWall.setRestitution(1f);
        leftWall.getRenderInfo().setColor(Color.GRAY);
        leftWall.setStatic(true);


        Body ceiling = new Body(new Transform(new Vector2(0,50)), 5f);
        ceiling.setCollider(new BoxCollider(ceiling, 50, 1.0f));
        ceiling.setRestitution(1f);
        ceiling.getRenderInfo().setColor(Color.GRAY);
        ceiling.setStatic(true);

        sim.addBody(floor);
        sim.addBody(rightWall);
        sim.addBody(leftWall);
        sim.addBody(ceiling);

        // ########################################

        Color[] colors = {Color.RED, Color.GREEN, Color.BLUE, Color.YELLOW, Color.CYAN};

        Vector2 initialVelocity = new Vector2(10, 0);
        Body b1 = new Body(new Transform(new Vector2(-10, 10)), 1f);
        b1.setCollider(new CircleCollider(b1, 0.1f));
        b1.setRestitution(1.2f);
        b1.setVelocity(initialVelocity);
        b1.getRenderInfo().setColor(colors[0]);

        Body b2 = new Body(new Transform(new Vector2(10, 10)), 1f);
        b2.setCollider(new CircleCollider(b2, 0.1f));
        b2.setRestitution(1.2f);
        b2.setVelocity(initialVelocity.mult(-1f));
        b2.getRenderInfo().setColor(colors[1]);


        sim.addBody(b1);
        sim.addBody(b2);


        return sim;
    });
}
