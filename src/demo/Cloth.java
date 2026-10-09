package demo;

import math.Transform;
import math.Vector2;
import physics.*;
import rendering.Renderer;
import simulation.Simulation;

import java.awt.Color;

/*
 * Three cloths made of the same mesh but different materials. Each is a grid of bodies joined by segments and
 * stretched between two posts, and a ball is dropped on the middle of each. Press Run to start.
 *
 *   left    canvas   rigid + dead:    barely stretches, holds its flat shape, swallows the ball's energy
 *   middle  rubber   soft + elastic:  stretches a lot, then springs back and throws the ball up (a trampoline)
 *   right   silk     soft + dead:     stretches, wraps around the ball and slowly settles without bouncing
 *
 * How a cloth is built (see buildCloth):
 *   - structural segments between neighbouring nodes resist stretching:       stretch rigidity / elasticity
 *   - shear segments across every cell stop the grid from collapsing sideways: shear rigidity / elasticity
 *     (they are not solid: they hold the cloth together but are not colliders and are not drawn)
 *   - optional bend joints between neighbouring segments resist folding:      bend rigidity / elasticity
 *
 * Rigidity 1 would make a part fully rigid; elasticity says how much of the stored energy comes back as motion.
 */
public class Cloth {
    // mesh: 13 x 4 nodes, 0.9 m apart, pinned along its left and right edge (the posts)
    private static final int COLUMNS = 13;
    private static final int ROWS = 4;
    private static final float SPACING = 0.9f;
    private static final float TOP_Y = 16f;

    private static final float NODE_MASS = 0.5f;
    private static final float THICKNESS = 0.18f;

    /** What a cloth is made of. A bend rigidity below 0 means no bend joints at all. */
    private record Material(String name,
                            float stretchRigidity, float stretchElasticity,
                            float shearRigidity, float shearElasticity,
                            float bendRigidity, float bendElasticity,
                            Color color) {}

    private static final Material CANVAS = new Material("canvas",
            0.95f, 0.3f,     // stretch: about 12 Hz, barely any bounce
            0.9f, 0.3f,      // shear: stiff
            0.85f, 0.4f,     // bend: resists folding, so it keeps its shape
            new Color(230, 160, 60));

    private static final Material RUBBER = new Material("rubber",
            0.55f, 0.95f,    // stretch: about 2.5 Hz, very bouncy
            0.5f, 0.95f,
            -1f, 0f,         // no bend joints
            new Color(70, 200, 90));

    private static final Material SILK = new Material("silk",
            0.8f, 0.15f,     // stretch: about 7 Hz but heavily damped
            0.35f, 0.15f,
            -1f, 0f,
            new Color(200, 90, 220));

    public static void main(String[] args) {
        Renderer.run(Cloth::buildScene);
    }

    public static Simulation buildScene() {
        Simulation sim = new Simulation(60);

        // a cloth doesn't need the 80 sweeps that tall stacks need, and it has hundreds of constraints
        sim.setSolverIterations(30);

        buildRoom(sim);

        buildCloth(sim, -12.5f, CANVAS);
        buildCloth(sim, 0f, RUBBER);
        buildCloth(sim, 12.5f, SILK);

        // the same ball over the middle of each cloth; no restitution of its own, so any bounce comes from the cloth
        for (float x : new float[]{-12.5f, 0f, 12.5f}) {
            Body ball = new Body(new Transform(new Vector2(x, TOP_Y + 6f)), 1f);
            ball.setCollider(new CircleCollider(ball, 1f));
            ball.setRestitution(0f);
            ball.getRenderInfo().setColor(Color.WHITE);
            sim.addBody(ball);
        }

        return sim;
    }

    // ---------------------------------------------------------------- the cloth

    private static void buildCloth(Simulation sim, float centerX, Material material) {
        float width = (COLUMNS - 1) * SPACING;

        // nodes: the first and last column are fixed posts, everything else can move
        Body[][] node = new Body[ROWS][COLUMNS];
        for (int row = 0; row < ROWS; row++) {
            for (int column = 0; column < COLUMNS; column++) {
                float x = centerX - width / 2 + column * SPACING;
                float y = TOP_Y - row * SPACING;

                boolean post = column == 0 || column == COLUMNS - 1;
                node[row][column] = post ? post(x, y) : node(x, y, material.color());
            }
        }

        // structural segments: along every row and every column
        Segment[][] horizontal = new Segment[ROWS][COLUMNS - 1];
        Segment[][] vertical = new Segment[ROWS - 1][COLUMNS];

        for (int row = 0; row < ROWS; row++) {
            for (int column = 0; column < COLUMNS - 1; column++) {
                horizontal[row][column] = link(sim, node[row][column], node[row][column + 1],
                        true, material.stretchRigidity(), material.stretchElasticity());
            }
        }
        for (int row = 0; row < ROWS - 1; row++) {
            for (int column = 0; column < COLUMNS; column++) {
                vertical[row][column] = link(sim, node[row][column], node[row + 1][column],
                        true, material.stretchRigidity(), material.stretchElasticity());
            }
        }

        // shear segments: both diagonals of every cell. Not solid: they brace the grid without being colliders
        for (int row = 0; row < ROWS - 1; row++) {
            for (int column = 0; column < COLUMNS - 1; column++) {
                link(sim, node[row][column], node[row + 1][column + 1],
                        false, material.shearRigidity(), material.shearElasticity());
                link(sim, node[row][column + 1], node[row + 1][column],
                        false, material.shearRigidity(), material.shearElasticity());
            }
        }

        // bend joints: keep neighbouring segments in line, so the cloth resists folding
        if (material.bendRigidity() >= 0) {
            for (int row = 0; row < ROWS; row++) {
                for (int column = 1; column < COLUMNS - 1; column++) {
                    bendJoint(sim, horizontal[row][column - 1], horizontal[row][column], material);
                }
            }
            for (int row = 1; row < ROWS - 1; row++) {
                for (int column = 1; column < COLUMNS - 1; column++) { // the posts don't move, nothing to bend
                    bendJoint(sim, vertical[row - 1][column], vertical[row][column], material);
                }
            }
        }
    }

    private static void bendJoint(Simulation sim, Segment first, Segment second, Material material) {
        SegmentJoint joint = new SegmentJoint(first, second); // rest angle: straight
        joint.setRigidity(material.bendRigidity());
        joint.setElasticity(material.bendElasticity());
        sim.addJoint(joint);
    }

    // ---------------------------------------------------------------- small helpers

    private static void buildRoom(Simulation sim) {
        Body floor = box(0, 0, 40, 1);
        floor.setStaticFriction(0.6f);
        floor.setDynamicFriction(0.4f);
        sim.addBody(floor);

        sim.addBody(box(19.5f, 14.5f, 1, 29));    // right wall
        sim.addBody(box(-19.5f, 14.5f, 1, 29));   // left wall
    }

    private static Body box(float x, float y, float width, float height) {
        Body body = new Body(new Transform(new Vector2(x, y)), 5f);
        body.setCollider(new BoxCollider(body, width, height));
        body.setRestitution(0f);
        body.getRenderInfo().setColor(Color.GRAY);
        body.setStatic(true);
        return body;
    }

    /** A fixed point of the cloth (a segment gives it a circle collider when it is attached). */
    private static Body post(float x, float y) {
        Body body = new Body(new Transform(new Vector2(x, y)), 5f);
        body.getRenderInfo().setColor(Color.DARK_GRAY);
        body.setStatic(true);
        return body;
    }

    /** A moving point of the cloth. The first segment that uses it adds it to the simulation. */
    private static Body node(float x, float y, Color color) {
        Body body = new Body(new Transform(new Vector2(x, y)), NODE_MASS);
        body.setRestitution(0f);
        body.getRenderInfo().setColor(color);
        return body;
    }

    private static Segment link(Simulation sim, Body a, Body b, boolean solid, float rigidity, float elasticity) {
        Segment segment = new Segment(a, b, THICKNESS);   // its rest length is the distance the bodies start with
        //segment.setSolid(solid);
        segment.setRigidity(rigidity);
        segment.setElasticity(elasticity);
        segment.getMiddleBody().getRenderInfo().setColor(b.getRenderInfo().getColor());
        sim.addSegment(segment);
        return segment;
    }
}