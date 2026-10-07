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
    light.getTransform().setScale(new Vector2(1f, 1f));
    light.getRenderInfo().setColor(Color.BLUE);
    light.setVelocity(new Vector2(-5f, 0f));

    Body medium = new Body(new Transform(new Vector2(0,25)), 10.0f);
    medium.getTransform().setScale(new Vector2(1f, 1f));
    medium.getRenderInfo().setColor(Color.RED);
    light.setVelocity(new Vector2(5f, 0f));

    sim.addBody(light);
    sim.addBody(medium);

    Renderer.run(() -> sim);
}
