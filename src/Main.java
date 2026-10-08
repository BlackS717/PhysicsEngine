import math.Transform;
import math.Vector2;
import physics.*;
import simulation.Simulation;
import rendering.Renderer;

import java.awt.*;

/*
 * Demonstration of every feature. The window shows 40 m x 28.5 m (800x600 at 20 px/m), so the room is 40 m wide
 * and everything below fits inside it. Press Run to start, Stop to pause, Reset to start over.
 *
 *   left to right, on the floor
 *     1. stack of 5 boxes             box-box collisions, stacking, friction (it must stay up)
 *     2. three bouncing balls         restitution: 0.9, 0.5 and 0.1
 *     3. pile of tilted boxes         rotation, rotated box-box collisions with 2 contact points, spin from impacts
 *     4. bullet between thin walls    continuous collision detection (a 140 m/s ball can't tunnel through 0.15 m walls)
 *     5. three square frames          deformation and shape recovery (springs + bend joints, see below)
 *     6. spring launcher              a compressed soft segment flinging a ball (rigidity + elasticity)
 *
 *   high up
 *     7. rigid chain                  segments with a fixed length, swinging from a static anchor
 *     8. suspended platform           soft segments hanging a rigid one; the ball hits near the left end, so the
 *                                     impact is shared unevenly between its two end bodies
 *     9. two cantilever beams         bend joints: welded (rigidity 1) vs soft (rigidity 0.5), same load
 *
 *   the three frames (5), left to right
 *     green  braced truss: 4 sides + 2 diagonals, all soft springs (the crossing diagonals ignore each other)
 *     blue   4 soft sides + soft corner joints: keeps its right angles
 *     red    4 soft sides only: no corner joints, so it collapses into a rhombus and stays that way
 */
void main() {
    Renderer.run(this::buildScene);
}

        Simulation buildScene() {
            Simulation sim = new Simulation(60);

            // continuous collision detection: set to false to watch the bullet (4) tunnel through the thin walls
            sim.setContinuousCollisionEnabled(true);

            buildRoom(sim);
            buildStack(sim);
            buildBouncingBalls(sim);
            buildTiltedPile(sim);
            buildBulletArena(sim);
            buildFrames(sim);
            buildLauncher(sim);
            buildChain(sim);
            buildSuspendedPlatform(sim);
            buildBeams(sim);

            return sim;
        }

// ---------------------------------------------------------------- the room

        void buildRoom(Simulation sim) {
            Body floor = staticBox(sim, 0, 0, 40, 1, Color.GRAY, 0f);
            floor.setStaticFriction(0.6f);
            floor.setDynamicFriction(0.4f);

            staticBox(sim, 19.5f, 14.5f, 1, 29, Color.GRAY, 0.5f);   // right wall (inner face at x = 19)
            staticBox(sim, -19.5f, 14.5f, 1, 29, Color.GRAY, 0.5f);  // left wall
            staticBox(sim, 0, 29, 40, 1, Color.GRAY, 0.5f);          // ceiling, just above the visible area
        }

// ---------------------------------------------------------------- 1. stacking

        void buildStack(Simulation sim) {
            Color[] colors = {Color.RED, Color.ORANGE, Color.YELLOW, Color.GREEN, Color.CYAN};

            for (int i = 0; i < 5; i++) {
                box(sim, -17f, 1.02f + i * 1.05f, 0f, 2.4f, 1f, 1f, 0.1f, colors[i]);
            }
        }

// ---------------------------------------------------------------- 2. restitution

        void buildBouncingBalls(Simulation sim) {
            ball(sim, -14f, 12f, 0.55f, 1f, 0.9f, Color.MAGENTA);  // bounces almost back to where it started
            ball(sim, -12.5f, 12f, 0.55f, 1f, 0.5f, Color.PINK);
            ball(sim, -11f, 12f, 0.55f, 1f, 0.1f, Color.LIGHT_GRAY); // barely bounces
        }

// ---------------------------------------------------------------- 3. rotation and box-box collisions

        void buildTiltedPile(Simulation sim) {
            box(sim, -8f, 3f, 0.5f, 2.2f, 1f, 1f, 0.2f, Color.RED);
            box(sim, -8.4f, 6f, -0.4f, 2.2f, 1f, 1f, 0.2f, Color.GREEN);
            box(sim, -7.6f, 9f, 0.8f, 2.2f, 1f, 1f, 0.2f, Color.BLUE);
        }

// ---------------------------------------------------------------- 4. continuous collision detection

        void buildBulletArena(Simulation sim) {
            staticBox(sim, -4.5f, 6f, 0.15f, 11f, Color.LIGHT_GRAY, 1f);
            staticBox(sim, 3.5f, 6f, 0.15f, 11f, Color.LIGHT_GRAY, 1f);

            // 140 m/s is 2.3 m per frame, far more than the wall is thick; no friction and no drag so it keeps its speed
            Body bullet = ball(sim, -3.5f, 6f, 0.3f, 0.3f, 1f, Color.ORANGE);
            bullet.setVelocity(new Vector2(140f, 3f));
            bullet.setStaticFriction(0f);
            bullet.setDynamicFriction(0f);
            bullet.setLinearDamping(0f);
        }

// ---------------------------------------------------------------- 5. deformation: truss / corner joints / nothing

        void buildFrames(Simulation sim) {
            buildFrame(sim, 6f, 14f, 0, new Color(60, 200, 60));    // braced truss
            buildFrame(sim, 9f, 14f, 1, new Color(70, 110, 255));   // corner joints
            buildFrame(sim, 12f, 14f, 2, new Color(240, 70, 70));   // sides only
        }

        /** mode 0: sides + diagonals, 1: sides + corner joints, 2: sides only. Tilted so a corner hits first. */
        void buildFrame(Simulation sim, float centerX, float centerY, int mode, Color color) {
            float side = 2f;
            float tilt = 0.5f;
            float halfDiagonal = side * 0.70710678f;

            Body[] corner = new Body[4];
            for (int i = 0; i < 4; i++) {
                double angle = Math.PI / 4 + i * Math.PI / 2 + tilt;
                corner[i] = node(centerX + halfDiagonal * (float) Math.cos(angle),
                        centerY + halfDiagonal * (float) Math.sin(angle), 1f, 0.1f, color);
            }

            // the sides are soft springs (rigidity 0.85 is about 8 Hz): they can be squashed and spring back
            Segment[] sides = new Segment[4];
            for (int i = 0; i < 4; i++) {
                sides[i] = link(sim, corner[i], corner[(i + 1) % 4], 0.25f, 0.85f, 0.7f);
            }

            if (mode == 0) {
                Segment diagonalA = link(sim, corner[0], corner[2], 0.2f, 0.8f, 0.7f);
                Segment diagonalB = link(sim, corner[1], corner[3], 0.2f, 0.8f, 0.7f);
                // the diagonals cross in the middle without being neighbours, so they must not collide with each other
                sim.ignoreCollisions(diagonalA.getMiddleBody(), diagonalB.getMiddleBody());
            }

            if (mode == 1) {
                for (int i = 0; i < 4; i++) {
                    SegmentJoint cornerJoint = new SegmentJoint(sides[i], sides[(i + 1) % 4]); // rest angle: 90 degrees
                    cornerJoint.setRigidity(0.65f);    // about 3.6 Hz angular spring
                    cornerJoint.setElasticity(0.8f);
                    sim.addJoint(cornerJoint);
                }
            }
        }

// ---------------------------------------------------------------- 6. spring launcher

        void buildLauncher(Simulation sim) {
            Body base = anchor(16.5f, 0.9f, Color.DARK_GRAY);

            // a box platform on top of the spring (a body that already has a collider keeps it)
            Body platform = new Body(new Transform(new Vector2(16.5f, 5.4f)), 1f);
            platform.setCollider(new BoxCollider(platform, 3f, 0.4f));
            platform.setRestitution(0f);
            platform.getRenderInfo().setColor(new Color(255, 190, 0));

            // rest length 6, but the platform starts only 4.5 above the base: the spring starts compressed by 1.5 m
            // rigidity 0.45 is about 1.6 Hz, elasticity 0.95 means it hardly loses energy: it flings the ball
            Segment spring = new Segment(base, platform, 6f, 0.3f);
            spring.setRigidity(0.45f);
            spring.setElasticity(0.95f);
            sim.addSegment(spring);

            ball(sim, 16.5f, 6.1f, 0.4f, 0.2f, 0.3f, Color.WHITE);
        }

// ---------------------------------------------------------------- 7. rigid chain

        void buildChain(Simulation sim) {
            // starts horizontal and swings down; every link has a fixed length (rigidity 1 is the default)
            Body previous = anchor(-12.5f, 25.5f, Color.DARK_GRAY);

            for (int i = 1; i <= 5; i++) {
                float x = -12.5f + 1.2f * i;
                Body next = (i < 5)
                        ? node(x, 25.5f, 0.5f, 0.1f, new Color(0, 170, 220))
                        : ball(sim, x, 25.5f, 0.6f, 3f, 0.1f, new Color(0, 170, 220)); // heavy bob at the end
                link(sim, previous, next, 0.3f, 0.8f, 1f);
                previous = next;
            }
        }

// ---------------------------------------------------------------- 8. soft suspension + impulse sharing

        void buildSuspendedPlatform(Simulation sim) {
            Body leftAnchor = anchor(-1.5f, 27.5f, Color.DARK_GRAY);
            Body rightAnchor = anchor(4.5f, 27.5f, Color.DARK_GRAY);

            Body left = node(-1.5f, 22.5f, 1f, 0.1f, Color.ORANGE);
            Body right = node(4.5f, 22.5f, 1f, 0.1f, Color.ORANGE);

            // two soft springs (about 1.9 Hz) hold the platform up
            Segment leftLink = link(sim, leftAnchor, left, 0.2f, 0.6f, 0.8f);
            Segment rightLink = link(sim, rightAnchor, right, 0.2f, 0.6f, 0.8f);

            sim.ignoreCollisions(leftLink.getMiddleBody());
            sim.ignoreCollisions(rightLink.getMiddleBody());

            // the platform itself is a rigid segment: a hit on it is shared between its two end bodies by where it lands
            Segment platform = link(sim, left, right, 0.5f, 1f, 0.8f);
            platform.getMiddleBody().getRenderInfo().setColor(Color.ORANGE);

            // lands about a sixth of the way along: the left end takes about five times as much of the impulse
            ball(sim, -0.5f, 26.5f, 0.5f, 15f, 0.3f, Color.WHITE);
        }

// ---------------------------------------------------------------- 9. bend joints: welded vs soft

        void buildBeams(Simulation sim) {
            buildBeam(sim, 21f, 1f, new Color(70, 110, 255));    // welded joints: stays straight under the load
            buildBeam(sim, 17.5f, 0.1f, new Color(240, 70, 70)); // soft joints (about 1.9 Hz): sags and bounces
        }

        void buildBeam(Simulation sim, float y, float jointRigidity, Color color) {
            Body previous = anchor(18.6f, y, Color.DARK_GRAY);
            Segment previousSegment = null;

            for (int i = 1; i <= 3; i++) {
                float x = 18.6f - 2f * i;
                Body next = (i < 3)
                        ? node(x, y, 0.5f, 0.1f, color)
                        : ball(sim, x, y, 0.5f, 2f, 0.1f, color);       // a heavy weight on the tip

                Segment segment = link(sim, previous, next, 0.3f, 1f, 0.8f);

                if (previousSegment != null) {
                    // the joint keeps the angle between neighbouring segments; the beam starts straight
                    SegmentJoint joint = new SegmentJoint(previousSegment, segment);
                    joint.setRigidity(jointRigidity);
                    joint.setElasticity(0.8f);
                    sim.addJoint(joint);
                }

                previous = next;
                previousSegment = segment;
            }
        }

// ---------------------------------------------------------------- small helpers

        Body staticBox(Simulation sim, float x, float y, float width, float height, Color color, float restitution) {
            Body body = new Body(new Transform(new Vector2(x, y)), 5f);
            body.setCollider(new BoxCollider(body, width, height));
            body.setRestitution(restitution);
            body.getRenderInfo().setColor(color);
            body.setStatic(true);
            sim.addBody(body);
            return body;
        }

        Body box(Simulation sim, float x, float y, float rotation, float width, float height,
                 float mass, float restitution, Color color) {
            Body body = new Body(new Transform(new Vector2(x, y), rotation), mass);
            body.setCollider(new BoxCollider(body, width, height));
            body.setRestitution(restitution);
            body.getRenderInfo().setColor(color);
            sim.addBody(body);
            return body;
        }

        Body ball(Simulation sim, float x, float y, float radius, float mass, float restitution, Color color) {
            Body body = new Body(new Transform(new Vector2(x, y)), mass);
            body.setCollider(new CircleCollider(body, radius));
            body.setRestitution(restitution);
            body.getRenderInfo().setColor(color);
            sim.addBody(body);
            return body;
        }

        /** A fixed point to hang segments from. A segment gives it a circle collider when it is attached. */
        Body anchor(float x, float y, Color color) {
            Body body = new Body(new Transform(new Vector2(x, y)), 5f);
            body.getRenderInfo().setColor(color);
            body.setStatic(true);
            return body;
        }

        /** A moving point for segments to connect. It is added to the simulation by the first segment that uses it. */
        Body node(float x, float y, float mass, float restitution, Color color) {
            Body body = new Body(new Transform(new Vector2(x, y)), mass);
            body.setRestitution(restitution);
            body.getRenderInfo().setColor(color);
            return body;
        }

        /** Connects two bodies with a segment (its length is their current distance) and adds it to the simulation. */
        Segment link(Simulation sim, Body a, Body b, float thickness, float rigidity, float elasticity) {
            Segment segment = new Segment(a, b, thickness);
            segment.setRigidity(rigidity);
            segment.setElasticity(elasticity);
            sim.addSegment(segment);
            return segment;
        }