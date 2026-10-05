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
    Vector2 initialVelocity = new Vector2(-5f, 0.0f);
    Body light = new Body(new Transform(new Vector2(-4,60)), 1.0f);
    light.getTransform().setScale(new Vector2(0.5f, 0.5f));
    light.setVelocity(initialVelocity);
    Body medium = new Body(new Transform(new Vector2(0,60)), 10.0f);
    medium.setVelocity(initialVelocity);
    medium.getTransform().setScale(new Vector2(1f, 1f));
    Body heavy = new Body(new Transform(new Vector2(4,60)), 100.0f);
    heavy.setVelocity(initialVelocity);
    heavy.getTransform().setScale(new Vector2(1.5f, 1.5f));


    sim.addBody(light);
    sim.addBody(medium);
    sim.addBody(heavy);
    Renderer.run(sim);
}
