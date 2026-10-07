package physics;

import math.Vector2;

/**
 * Convention used everywhere: a Collision(A, B, normal, penetration, contactPoint) has a normal pointing from A to B.
 * Boxes may be rotated: their rotation is read from the owning body's transform (radians, counter-clockwise).
 */
public class CollisionDetector {

    // when two axes overlap almost equally, keep the earlier one (A's faces first) so the contact doesn't flicker
    private static final float AXIS_TOLERANCE = 0.0005f;

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
            return new Collision(collider1, collider2, c.getNormal().mult(-1), c.getPenetration(), c.getContactPoint());
        }
        else if (collider1 instanceof BoxCollider && collider2 instanceof BoxCollider) {
            return detectBoxBoxCollision((BoxCollider) collider1, (BoxCollider) collider2);
        }

        return null; // unsupported collider type
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

        // middle of the overlap region
        Vector2 contactPoint = circle1.getCenter().add(normal.mult(circle1.getRadius() - penetration / 2f));

        return new Collision(circle1, circle2, normal, penetration, contactPoint);
    }

    // ---------------------------------------------------------------- circle vs (rotated) box

    private static Collision detectCircleBoxCollision(CircleCollider circle, BoxCollider rectangle) {
        Vector2 circleCenter = circle.getCenter();
        float radius = circle.getRadius();

        float angle = rectangle.getBody().getTransform().getRotation();
        float halfWidth = rectangle.getWidth() / 2;
        float halfHeight = rectangle.getHeight() / 2;

        // work in the box's local space, where the box is axis-aligned and centred on the origin
        Vector2 local = rotate(circleCenter.sub(rectangle.getCenter()), -angle);

        Vector2 closestLocal = new Vector2(
                clamp(local.getX(), -halfWidth, halfWidth),
                clamp(local.getY(), -halfHeight, halfHeight)
        );

        Vector2 diff = local.sub(closestLocal);
        float distanceSquared = diff.dotProduct(diff); // real squared distance

        if (distanceSquared > radius * radius) {
            return null;
        }

        Vector2 normalLocal; // circle -> box, in local space
        float penetration;

        if (distanceSquared == 0) {
            // circle centre inside the box (or exactly on its boundary): push out through the nearest edge
            float[] d = {
                    local.getX() + halfWidth,  // left
                    halfWidth - local.getX(),  // right
                    halfHeight - local.getY(), // top
                    local.getY() + halfHeight  // bottom
            };
            int nearest = nearestEdgeIndex(d);

            Vector2 outward;
            switch (nearest) {
                case 0:  outward = new Vector2(-1, 0); break; // left
                case 1:  outward = new Vector2(1, 0);  break; // right
                case 2:  outward = new Vector2(0, 1);  break; // top
                default: outward = new Vector2(0, -1); break; // bottom
            }

            normalLocal = outward.mult(-1);
            penetration = radius + d[nearest];
        } else {
            float distance = (float) Math.sqrt(distanceSquared);

            normalLocal = closestLocal.sub(local).div(distance);
            penetration = radius - distance;
        }

        // back to world space
        Vector2 normal = rotate(normalLocal, angle);
        Vector2 contactPoint = circleCenter.add(normal.mult(radius - penetration / 2f));

        return new Collision(circle, rectangle, normal, penetration, contactPoint);
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

    // ---------------------------------------------------------------- box vs box (rotated, SAT)

    /**
     * Separating Axis Theorem for two oriented boxes: test the 4 face normals (2 per box). If any axis separates
     * the boxes there is no collision; otherwise the axis with the smallest overlap is the collision normal.
     * The contact point is found by taking the incident box's vertices that sank past the reference face.
     */
    private static Collision detectBoxBoxCollision(BoxCollider a, BoxCollider b) {
        Vector2 centerA = a.getCenter();
        Vector2 centerB = b.getCenter();

        float angleA = a.getBody().getTransform().getRotation();
        float angleB = b.getBody().getTransform().getRotation();

        float hwA = a.getWidth() / 2, hhA = a.getHeight() / 2;
        float hwB = b.getWidth() / 2, hhB = b.getHeight() / 2;

        // local x / y axes of each box, in world space
        Vector2 axA = new Vector2((float) Math.cos(angleA), (float) Math.sin(angleA));
        Vector2 ayA = new Vector2(-axA.getY(), axA.getX());
        Vector2 axB = new Vector2((float) Math.cos(angleB), (float) Math.sin(angleB));
        Vector2 ayB = new Vector2(-axB.getY(), axB.getX());

        Vector2[] axes = {axA, ayA, axB, ayB};
        Vector2 delta = centerB.sub(centerA);

        float minOverlap = Float.MAX_VALUE;
        int bestIndex = -1;

        for (int i = 0; i < axes.length; i++) {
            Vector2 axis = axes[i];

            // how far each box extends along this axis from its centre
            float radiusA = hwA * Math.abs(axis.dotProduct(axA)) + hhA * Math.abs(axis.dotProduct(ayA));
            float radiusB = hwB * Math.abs(axis.dotProduct(axB)) + hhB * Math.abs(axis.dotProduct(ayB));

            float overlap = radiusA + radiusB - Math.abs(delta.dotProduct(axis));

            if (overlap < 0) {
                return null; // found a separating axis
            }

            if (overlap < minOverlap - AXIS_TOLERANCE) {
                minOverlap = overlap;
                bestIndex = i;
            }
        }

        // collision normal: the best axis, oriented from A to B
        Vector2 normal = axes[bestIndex];
        if (delta.dotProduct(normal) < 0) {
            normal = normal.mult(-1);
        }

        // reference box owns the face the other box pushed into; the incident box is the other one
        boolean referenceIsA = bestIndex < 2;
        Vector2 referenceCenter = referenceIsA ? centerA : centerB;
        Vector2 referenceNormal = referenceIsA ? normal : normal.mult(-1); // points out of the reference box

        float referenceExtent;  // half size of the reference box along its face normal
        float tangentExtent;    // half size of the reference face along its own length
        switch (bestIndex) {
            case 0:  referenceExtent = hwA; tangentExtent = hhA; break;
            case 1:  referenceExtent = hhA; tangentExtent = hwA; break;
            case 2:  referenceExtent = hwB; tangentExtent = hhB; break;
            default: referenceExtent = hhB; tangentExtent = hwB; break;
        }

        Vector2 tangent = new Vector2(-referenceNormal.getY(), referenceNormal.getX());
        float planeOffset = referenceCenter.dotProduct(referenceNormal) + referenceExtent;

        Vector2[] incidentVertices = referenceIsA
                ? getVertices(centerB, axB, ayB, hwB, hhB)
                : getVertices(centerA, axA, ayA, hwA, hhA);

        // contact point = average of the incident vertices that are past the reference face
        Vector2 sum = new Vector2(0, 0);
        int count = 0;

        for (Vector2 v : incidentVertices) {
            float depth = planeOffset - v.dotProduct(referenceNormal);
            if (depth < 0) continue; // this vertex is still outside the reference face

            // keep the point within the reference face's length
            float lateral = v.sub(referenceCenter).dotProduct(tangent);
            float clampedLateral = clamp(lateral, -tangentExtent, tangentExtent);

            Vector2 point = v
                    .add(tangent.mult(clampedLateral - lateral))
                    .add(referenceNormal.mult(depth / 2f)); // halfway between the vertex and the face

            sum = sum.add(point);
            count++;
        }

        Vector2 contactPoint = count > 0 ? sum.div(count) : centerA.add(delta.mult(0.5f));

        return new Collision(a, b, normal, minOverlap, contactPoint);
    }

    private static Vector2[] getVertices(Vector2 center, Vector2 axisX, Vector2 axisY, float halfWidth, float halfHeight) {
        Vector2 x = axisX.mult(halfWidth);
        Vector2 y = axisY.mult(halfHeight);

        return new Vector2[]{
                center.add(x).add(y),
                center.sub(x).add(y),
                center.sub(x).sub(y),
                center.add(x).sub(y)
        };
    }

    // ---------------------------------------------------------------- helpers

    /** Rotates v counter-clockwise by angle (radians). */
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