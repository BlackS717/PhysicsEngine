package rendering;

import math.Vector2;
import physics.Body;
import simulation.Simulation;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;
import javax.swing.Timer;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;

/**
 * Simple Swing renderer that steps a Simulation at its fps and draws every body as a circle.
 * World space: meters, y pointing up, origin at the bottom-center of the window.
 */
public class Renderer extends JPanel {
    private final Simulation simulation;
    private final Timer timer;

    private final float pixelsPerMeter;
    private final float bodyRadiusMeters = 1.0f;

    public Renderer(Simulation simulation, int width, int height, float pixelsPerMeter) {
        this.simulation = simulation;
        this.pixelsPerMeter = pixelsPerMeter;

        setPreferredSize(new Dimension(width, height));
        setBackground(Color.BLACK);

        int delayMs = Math.max(1, 1000 / simulation.getFps());
        this.timer = new Timer(delayMs, e -> {
            simulation.step();
            repaint();
        });
    }

    public void start() {
        timer.start();
    }

    public void stop() {
        timer.stop();
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);

        Graphics2D g2 = (Graphics2D) g;
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        for (Body body : simulation.getBodies()) {
            if (body == null) continue;

            g2.setColor(body.getRenderInfo().getColor());
            Vector2 pos = body.getTransform().getPosition();
            Vector2 scale = body.getTransform().getScale();

            int width = Math.round(bodyRadiusMeters * pixelsPerMeter * scale.getX());
            int height = Math.round(bodyRadiusMeters * pixelsPerMeter * scale.getY());

            int x = Math.round(getWidth() / 2f + pos.getX() * pixelsPerMeter);
            int y = Math.round(getHeight() - pos.getY() * pixelsPerMeter); // flip y: world up -> screen up

            g2.fillOval(x - width, y - height, width * 2, height * 2);
        }
    }

    /** Opens a window and runs the simulation in it. */
    public static void run(Simulation simulation) {
        SwingUtilities.invokeLater(() -> {
            Renderer renderer = new Renderer(simulation, 800, 600, 20f);

            JFrame frame = new JFrame("Simulation");
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            frame.add(renderer);
            frame.pack();
            frame.setLocationRelativeTo(null);
            frame.setVisible(true);

            renderer.start();
        });
    }
}