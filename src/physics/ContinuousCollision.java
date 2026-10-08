package physics;

import math.Vector2;

/**
 * Continuous collision detection (CCD): finds WHEN two bodies first touch during a step, so a fast body can't
 * jump over a thin one between two frames.
 *
 * The motion of each body over the step is treated as a straight line from the pose it had before integrate()
 * to the pose it has now (position and rotation both interpolated linearly). The time of impact is found with
 * conservative advancement: measure the gap between the shapes, then move forward in time by the largest amount
 * that is guaranteed not to close that gap, and repeat until the shapes touch or the step ends.
 */
public final class ContinuousCollision {
    /** A gap smaller than this counts as touching. */
    public static final float TOLERANCE = 0.02f;

    /**
     * Margin the detector must use for pairs that CCD rewound, so the contact is found even though the
     * bodies are (just) not overlapping. Must be larger than TOLERANCE.
     */
    public static final float MARGIN = 0.05f;

    private static final int MAX_ITERATIONS = 64;

    // a pair only needs CCD if it moves more than this fraction of the thinner shape's half-size per step
    private static final float MOTION_FRACTION = 0.5f;

    private ContinuousCollision() {}

    /**
     * @return the time of impact as a fraction of the last step (0 = start of the step, 1 = end),
     *         or -1 if the two bodies don't hit each other during the step (or don't need CCD).
     */
    public static float timeOfImpact(Body a, Body b) {
        if (a == null || b == null || a.getCollider() == null || b.getCollider() == null) return -1f;

        Sweep sa = Sweep.of(a);
        Sweep sb = Sweep.of(b);
        if (sa == null || sb == null) return -1f;

        Vector2 relativeDisplacement = sb.end.sub(sb.start).sub(sa.end.sub(sa.start));

        // upper bound on how fast the gap between the two shapes can shrink (per unit of step time):
        // their relative travel, plus how far the farthest point of each shape can swing because of rotation
        float motionBound = relativeDisplacement.magnitude()
                + Math.abs(sa.endAngle - sa.startAngle) * sa.boundingRadius
                + Math.abs(sb.endAngle - sb.startAngle) * sb.boundingRadius;

        // normal-speed pairs are handled fine by the discrete detector
        if (motionBound <= MOTION_FRACTION * Math.min(sa.thickness, sb.thickness)) return -1f;
        if (motionBound < 1e-9f) return -1f;

        // cheap rejection: do the bounding circles ever come close enough to touch?
        float reach = sa.boundingRadius + sb.boundingRadius + MARGIN;
        if (closestApproach(sb.start.sub(sa.start), relativeDisplacement) > reach) return -1f;

        float t = 0f;
        for (int i = 0; i < MAX_ITERATIONS; i++) {
            Separation sep = separation(sa, sb, t);

            if (sep.distance <= TOLERANCE) {
                if (t > 0f) {
                    return t; // we advanced up to the contact
                }
                // already touching at the start of the step: only a hit if they are moving together
                float closing = -relativeDisplacement.dotProduct(sep.normal);
                return closing > 1e-4f ? 0f : -1f;
            }

            t += sep.distance / motionBound;
            if (t >= 1f) return -1f;
        }

        return -1f; // couldn't prove a contact within the iteration budget
    }

    // ---------------------------------------------------------------- shapes in motion

    /** One body's shape together with its pose at the start and at the end of the step. */
    private static final class Sweep {
        final boolean circle;
        final float radius;        // circle only
        final float halfWidth;     // box only
        final float halfHeight;    // box only
        final float boundingRadius;
        final float thickness;     // half-size of the thinnest dimension

        final Vector2 start;
        final Vector2 end;
        final float startAngle;
        final float endAngle;

        private Sweep(boolean circle, float radius, float halfWidth, float halfHeight,
                      Vector2 start, Vector2 end, float startAngle, float endAngle) {
            this.circle = circle;
            this.radius = radius;
            this.halfWidth = halfWidth;
            this.halfHeight = halfHeight;
            this.boundingRadius = circle ? radius : (float) Math.sqrt(halfWidth * halfWidth + halfHeight * halfHeight);
            this.thickness = circle ? radius : Math.min(halfWidth, halfHeight);
            this.start = start;
            this.end = end;
            this.startAngle = startAngle;
            this.endAngle = endAngle;
        }

        static Sweep of(Body body) {
            Collider collider = body.getCollider();

            // bodies that can't move have no previous pose: they simply stay where they are
            boolean movable = body.getInverseMass() != 0;
            Vector2 end = body.getTransform().getPosition();
            Vector2 start = movable ? body.getPreviousPosition() : end;
            float endAngle = body.getTransform().getRotation();
            float startAngle = movable ? body.getPreviousRotation() : endAngle;

            if (collider instanceof CircleCollider) {
                return new Sweep(true, ((CircleCollider) collider).getRadius(), 0, 0,
                        start, end, startAngle, endAngle);
            }
            if (collider instanceof BoxCollider) {
                BoxCollider box = (BoxCollider) collider;
                return new Sweep(false, 0, box.getWidth() / 2, box.getHeight() / 2,
                        start, end, startAngle, endAngle);
            }
            return null; // unsupported collider type
        }

        Vector2 position(float t) {
            return start.add(end.sub(start).mult(t));
        }

        float angle(float t) {
            return startAngle + (endAngle - startAngle) * t;
        }
    }

    // ---------------------------------------------------------------- gap between two shapes

    /** Lower bound of the distance between two shapes (negative when they overlap) and the direction a -> b. */
    private static final class Separation {
        final float distance;
        final Vector2 normal;

        Separation(float distance, Vector2 normal) {
            this.distance = distance;
            this.normal = normal;
        }
    }

    private static Separation separation(Sweep a, Sweep b, float t) {
        Vector2 pa = a.position(t);
        Vector2 pb = b.position(t);

        if (a.circle && b.circle) {
            return circleCircle(a, pa, b, pb);
        } else if (a.circle) {
            return circleBox(a, pa, b, pb, b.angle(t));
        } else if (b.circle) {
            Separation s = circleBox(b, pb, a, pa, a.angle(t));
            return new Separation(s.distance, s.normal.mult(-1));
        } else {
            return boxBox(a, pa, a.angle(t), b, pb, b.angle(t));
        }
    }

    private static Separation circleCircle(Sweep a, Vector2 pa, Sweep b, Vector2 pb) {
        Vector2 delta = pb.sub(pa);
        float distance = delta.magnitude();
        Vector2 normal = distance == 0 ? new Vector2(0, 1) : delta.div(distance);

        return new Separation(distance - a.radius - b.radius, normal);
    }

    /** Normal points from the circle to the box. */
    private static Separation circleBox(Sweep circle, Vector2 circleCenter, Sweep box, Vector2 boxCenter, float boxAngle) {
        // work in the box's local space, where it is axis-aligned and centred on the origin
        Vector2 local = rotate(circleCenter.sub(boxCenter), -boxAngle);

        Vector2 closest = new Vector2(
                clamp(local.getX(), -box.halfWidth, box.halfWidth),
                clamp(local.getY(), -box.halfHeight, box.halfHeight)
        );

        Vector2 diff = local.sub(closest);
        float distance = diff.magnitude();

        if (distance == 0) {
            // circle centre inside the box
            Vector2 toBox = boxCenter.sub(circleCenter);
            Vector2 normal = toBox.magnitude() == 0 ? new Vector2(0, 1) : toBox.normalize();
            return new Separation(-circle.radius, normal);
        }

        Vector2 normalLocal = closest.sub(local).div(distance); // circle -> box
        return new Separation(distance - circle.radius, rotate(normalLocal, boxAngle));
    }

    /**
     * Largest gap over the four face-normal axes. That is a lower bound of the true distance between the boxes
     * (and negative when no axis separates them), which is what conservative advancement needs.
     */
    private static Separation boxBox(Sweep a, Vector2 pa, float angleA, Sweep b, Vector2 pb, float angleB) {
        Vector2 axA = new Vector2((float) Math.cos(angleA), (float) Math.sin(angleA));
        Vector2 ayA = new Vector2(-axA.getY(), axA.getX());
        Vector2 axB = new Vector2((float) Math.cos(angleB), (float) Math.sin(angleB));
        Vector2 ayB = new Vector2(-axB.getY(), axB.getX());

        Vector2[] axes = {axA, ayA, axB, ayB};
        Vector2 delta = pb.sub(pa);

        float bestGap = -Float.MAX_VALUE;
        Vector2 bestNormal = axA;

        for (Vector2 axis : axes) {
            float radiusA = a.halfWidth * Math.abs(axis.dotProduct(axA)) + a.halfHeight * Math.abs(axis.dotProduct(ayA));
            float radiusB = b.halfWidth * Math.abs(axis.dotProduct(axB)) + b.halfHeight * Math.abs(axis.dotProduct(ayB));

            float projection = delta.dotProduct(axis);
            float gap = Math.abs(projection) - radiusA - radiusB;

            if (gap > bestGap) {
                bestGap = gap;
                bestNormal = projection < 0 ? axis.mult(-1) : axis;
            }
        }

        return new Separation(bestGap, bestNormal);
    }

    // ---------------------------------------------------------------- helpers

    /** Smallest distance from the origin to the segment start -> start + motion. */
    private static float closestApproach(Vector2 start, Vector2 motion) {
        float lengthSquared = motion.dotProduct(motion);
        if (lengthSquared == 0) return start.magnitude();

        float t = clamp(-start.dotProduct(motion) / lengthSquared, 0f, 1f);
        return start.add(motion.mult(t)).magnitude();
    }

    private static Vector2 rotate(Vector2 v, float angle) {
        float cos = (float) Math.cos(angle);
        float sin = (float) Math.sin(angle);
        return new Vector2(
                v.getX() * cos - v.getY() * sin,
                v.getX() * sin + v.getY() * cos
        );
    }

    private static float clamp(float value, float min, float max) {
        return Math.max(min, Math.min(max, value));
    }
}