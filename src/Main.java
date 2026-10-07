import math.Transform;
import math.Vector2;
import physics.Body;
import simulation.Simulation;
import rendering.Renderer;

import java.awt.*;

//TIP To <b>Run</b> code, press <shortcut actionId="Run"/> or
// click the <icon src="AllIcons.Actions.Execute"/> icon in the gutter.
void main() {
    Simulation sim = new Simulation(60);

    Body light = new Body(new Transform(new Vector2(-4,25)), 5.0f);
    light.getRenderInfo().setColor(Color.BLUE);
    light.setRestitution(0f);
//    light.setVelocity(new Vector2(5f, 0f));

    Body medium = new Body(new Transform(new Vector2(-3,0)), 5f);
    medium.getRenderInfo().setColor(Color.RED);
    medium.setStatic(true);
    medium.setRestitution(0.5f);
    medium.getTransform().setScale(new Vector2(10, 1));


    sim.addBody(light);
    sim.addBody(medium);

    Renderer.run(() -> sim);
}
