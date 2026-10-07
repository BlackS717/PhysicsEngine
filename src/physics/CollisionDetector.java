package physics;

import math.Vector2;

/**
 * Convention used everywhere: a Collision(A, B, normal, penetration) has a normal pointing from A to B.
 */
public class CollisionDetector {

    public static Collision detectCollision(Collider collider1, Collider collider2) {
        if (collider1 instanceof CircleCollider && collider2 instanceof CircleCollider) {
            return detectCircleCollision((CircleCollider) collider1, (CircleCollider) collider2);
        }
        else if (collider1 instanceof CircleCollider && collider2 instanceof BoxCollider) {
            return detectCircleBoxCollision((CircleCollider) collider1, (BoxCollider) collider2);
        }
        else if (collider1 instanceof BoxCollider && collider2 instanceof CircleCollider) {
            Collision c = detectCircleBoxCollision((CircleCollider) collider2, (BoxCollider) collider1);
            if (c == null) return null;

            // re-express the result in the caller's order: (box, circle), normal box -> circle
            return new Collision(collider1, collider2, c.getNormal().mult(-1), c.getPenetration());
        }

        return null; // unsupported pair (e.g. box vs box)
    }

    // ---------------------------------------------------------------- circle vs circle

    private static Collision detectCircleCollision(CircleCollider circle1, CircleCollider circle2) {
        Vector2 distanceVector = circle2.getCenter().sub(circle1.getCenter());
        float distance = distanceVector.magnitude();
        float radiusSum = circle1.getRadius() + circle2.getRadius();

        if (distance > radiusSum) {
            return null;
        }

        // identical centers -> no direction, pick an arbitrary one so the bodies still separate
        Vector2 normal = distance == 0 ? new Vector2(0, 1) : distanceVector.div(distance);
        float penetration = radiusSum - distance;

        return new Collision(circle1, circle2, normal, penetration);
    }

    // ---------------------------------------------------------------- circle vs box

    private static Collision detectCircleBoxCollision(CircleCollider circle, BoxCollider rectangle) {
        Vector2 circleCenter = circle.getCenter();
        float circleRadius = circle.getRadius();

        Vector2 closestPoint = getClosestPoint(rectangle, circleCenter);
        Vector2 distanceVector = circleCenter.sub(closestPoint);

        // real squared distance (mult(Vector2) is component-wise, so it can't be used here)
        float distanceSquared = distanceVector.dotProduct(distanceVector);

        if (distanceSquared > circleRadius * circleRadius) {
            return null;
        }

        // centre inside the box, or exactly on its boundary (distance 0 -> no usable direction)
        boolean centerInside = distanceSquared == 0 || rectangle.contains(circleCenter);

        Vector2 normal;
        float penetration;

        if (centerInside) {
            float[] d = getDistanceToEdge(circle, rectangle); // left, right, top, bottom
            int nearest = nearestEdgeIndex(d);

            Vector2 outward;
            switch (nearest) {
                case 0:  outward = new Vector2(-1, 0); break; // left
                case 1:  outward = new Vector2(1, 0);  break; // right
                case 2:  outward = new Vector2(0, 1);  break; // top
                default: outward = new Vector2(0, -1); break; // bottom
            }

            normal = outward.mult(-1);              // circle -> box
            penetration = circleRadius + d[nearest];
        } else {
            float distance = (float) Math.sqrt(distanceSquared);

            normal = closestPoint.sub(circleCenter).div(distance); // circle -> box
            penetration = circleRadius - distance;
        }

        return new Collision(circle, rectangle, normal, penetration);
    }

    private static Vector2 getClosestPoint(BoxCollider rectangle, Vector2 circleCenter) {
        Vector2 rectCenter = rectangle.getCenter();

        float halfWidth = rectangle.getWidth() / 2;
        float halfHeight = rectangle.getHeight() / 2;

        float closestX = clamp(circleCenter.getX(), rectCenter.getX() - halfWidth, rectCenter.getX() + halfWidth);
        float closestY = clamp(circleCenter.getY(), rectCenter.getY() - halfHeight, rectCenter.getY() + halfHeight);

        return new Vector2(closestX, closestY);
    }

    /** Distances from the circle's centre to the box edges: {left, right, top, bottom}. */
    private static float[] getDistanceToEdge(CircleCollider circle, BoxCollider rectangle) {
        Vector2 c = rectangle.getCenter();
        float halfWidth = rectangle.getWidth() / 2;
        float halfHeight = rectangle.getHeight() / 2;

        float left = c.getX() - halfWidth;
        float right = c.getX() + halfWidth;
        float top = c.getY() + halfHeight;
        float bottom = c.getY() - halfHeight;

        float cx = circle.getCenter().getX();
        float cy = circle.getCenter().getY();

        return new float[]{
                Math.abs(cx - left),
                Math.abs(right - cx),
                Math.abs(top - cy),
                Math.abs(cy - bottom)
        };
    }

    /** Index of the smallest distance; on ties the first one wins (left, right, top, bottom). */
    private static int nearestEdgeIndex(float[] distances) {
        int best = 0;
        for (int i = 1; i < distances.length; i++) {
            if (distances[i] < distances[best]) {
                best = i;
            }
        }
        return best;
    }

    private static float clamp(float value, float min, float max) {
        return Math.max(min, Math.min(max, value));
    }
}