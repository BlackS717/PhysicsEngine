package rendering;

import math.Vector2;
import physics.Body;
import physics.BoxCollider;
import physics.CircleCollider;
import physics.Collider;
import simulation.Simulation;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;
import javax.swing.Timer;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.GraphicsEnvironment;
import java.awt.Rectangle;
import java.awt.RenderingHints;
import java.util.function.Supplier;

/**
 * Simple Swing renderer that steps a Simulation at its fps and draws every body.
 * World space: meters, y pointing up, origin at the bottom-center of the view (on the ground line).
 * Rotation: radians, counter-clockwise (y up), 0 = pointing along +x.
 */
public class Renderer extends JPanel {
    private static final int GROUND_MARGIN_PX = 30;

    private final Supplier<Simulation> simulationFactory;
    private final Timer timer;
    private Simulation simulation;

    private final float pixelsPerMeter;

    /**
     * @param simulationFactory builds a fresh simulation in its initial state; called once now and again on every reset
     */
    public Renderer(Supplier<Simulation> simulationFactory, int width, int height, float pixelsPerMeter) {
        this.simulationFactory = simulationFactory;
        this.simulation = simulationFactory.get();
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

    /** Stops the simulation and restores its initial state. */
    public void reset() {
        timer.stop();
        simulation = simulationFactory.get();
        repaint();
    }

    public boolean isRunning() {
        return timer.isRunning();
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);

        Graphics2D g2 = (Graphics2D) g;
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        int originX = getWidth() / 2;
        int originY = getHeight() - GROUND_MARGIN_PX;

        // ground line at y = 0
        g2.setColor(Color.DARK_GRAY);
        g2.drawLine(0, originY, getWidth(), originY);

        for (Body body : simulation.getBodies()) {
            drawBody(g2, body, originX, originY, pixelsPerMeter);
        }
    }

    private static void drawBody(Graphics2D g, Body body, int originX, int originY, float pixelsPerMeter) {
        if (body == null) return;

        Collider collider = body.getCollider();
        if (collider == null) return;

        Vector2 pos = body.getTransform().getPosition();
        float angle = body.getTransform().getRotation();

        int x = Math.round(originX + pos.getX() * pixelsPerMeter);
        int y = Math.round(originY - pos.getY() * pixelsPerMeter); // flip y: world up -> screen up

        // work on a copy so the translate/rotate never leaks into other bodies
        Graphics2D g2 = (Graphics2D) g.create();
        try {
            g2.translate(x, y);
            g2.rotate(-angle); // minus: screen y is flipped, so counter-clockwise becomes negative

            Color color = body.getRenderInfo().getColor();
            g2.setColor(color);

            if (collider instanceof CircleCollider) {
                int r = Math.round(((CircleCollider) collider).getRadius() * pixelsPerMeter);

                g2.fillOval(-r, -r, r * 2, r * 2);

                // radius line so the rotation is visible on a circle
                g2.setColor(contrastColor(color));
                g2.drawLine(0, 0, r, 0);

            } else if (collider instanceof BoxCollider) {
                int halfW = Math.round(((BoxCollider) collider).getWidth() / 2 * pixelsPerMeter);
                int halfH = Math.round(((BoxCollider) collider).getHeight() / 2 * pixelsPerMeter);

                g2.fillRect(-halfW, -halfH, halfW * 2, halfH * 2);
            }
        } finally {
            g2.dispose();
        }
    }

    /** Black on bright colours, white on dark ones, so the rotation marker is always visible. */
    private static Color contrastColor(Color c) {
        int luminance = (c.getRed() * 299 + c.getGreen() * 587 + c.getBlue() * 114) / 1000;
        return luminance > 128 ? Color.BLACK : Color.WHITE;
    }

    /**
     * Opens a window with Run / Stop / Reset buttons.
     * The factory must return a new simulation in its starting state each time it is called.
     */
    public static void run(Supplier<Simulation> simulationFactory) {
        SwingUtilities.invokeLater(() -> {
            Renderer renderer = new Renderer(simulationFactory, 1200, 900, 20f);

            JPanel controls = getControls(renderer);

            JFrame frame = new JFrame("Simulation");
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            frame.setLayout(new BorderLayout());
            frame.add(renderer, BorderLayout.CENTER);
            frame.add(controls, BorderLayout.SOUTH);
            frame.pack();

            // keep the whole window (including the buttons) on screen
            Rectangle screen = GraphicsEnvironment.getLocalGraphicsEnvironment().getMaximumWindowBounds();
            frame.setSize(
                    Math.min(frame.getWidth(), screen.width),
                    Math.min(frame.getHeight(), screen.height)
            );
            frame.setLocationRelativeTo(null);
            frame.setVisible(true);
        });
    }

    private static JPanel getControls(Renderer renderer) {
        JButton runButton = new JButton("Run");
        JButton stopButton = new JButton("Stop");
        JButton resetButton = new JButton("Reset");
        stopButton.setEnabled(false);

        runButton.addActionListener(e -> {
            renderer.start();
            runButton.setEnabled(false);
            stopButton.setEnabled(true);
        });
        stopButton.addActionListener(e -> {
            renderer.stop();
            runButton.setEnabled(true);
            stopButton.setEnabled(false);
        });
        resetButton.addActionListener(e -> {
            renderer.reset();
            runButton.setEnabled(true);
            stopButton.setEnabled(false);
        });

        JPanel controls = new JPanel(new FlowLayout(FlowLayout.CENTER, 10, 6));
        controls.add(runButton);
        controls.add(stopButton);
        controls.add(resetButton);
        return controls;
    }
}