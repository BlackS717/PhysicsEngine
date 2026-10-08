import math.Transform;
import math.Vector2;
import physics.Body;
import physics.BoxCollider;
import physics.CircleCollider;
import physics.Segment;
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

        float startingY = 20f;

        Body anchor = new Body(new Transform(new Vector2(0, startingY)), 5f);
        anchor.setRestitution(0f);
//        anchor.setStatic(true);

        Body previous = anchor;
        for (int i = 0; i < 16; i++) {
            Body next = new Body(new Transform(new Vector2(1.1f * (i + 1), startingY)), 1f);
            next.setRestitution(0f);
            next.getRenderInfo().setColor(colors[i % colors.length]);
            sim.addSegment(new Segment(previous, next, 0.3f));   // thickness 0.6
            previous = next;
        }

        // add a static ball below the chain to collide with
        Body ball = new Body(new Transform(new Vector2(5, 10)), 5f);
        ball.setStatic(true);
        ball.setCollider(new CircleCollider(ball, 1.0f));
        ball.setRestitution(0f);
        ball.getRenderInfo().setColor(Color.MAGENTA);
        sim.addBody(ball);

        return sim;
    });
}
